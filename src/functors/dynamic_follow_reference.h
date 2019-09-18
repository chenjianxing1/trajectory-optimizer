
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"
#include "src/commons/commons.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Line;
using commons::Parameters;
using commons::DistanceTrajetory;
using optimizer::BaseFunctor;
using dynamics::GenerateDynamicTrajectory;
using dynamics::SingleTrackModel;
using dynamics::integrationRK4;
using dynamics::integrationEuler;


class DynamicModelFollowReference : public BaseFunctor {
 public:
  DynamicModelFollowReference() : BaseFunctor(nullptr) {}
  explicit DynamicModelFollowReference(Matrix_t<double> initial_state,
                           Parameters* params) :
    initial_state_(initial_state),
    BaseFunctor(params) {}
  virtual ~DynamicModelFollowReference() {}

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals, T cost = T(0.0)) {
    // Convert to Eigen
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    // Generate dynamic trajectory
    Matrix_t<T> trajectory =
      GenerateDynamicTrajectory<T, SingleTrackModel<T, integrationRK4>>(
        initial_state_t, opt_vec, *params_);

    // Distance
    Matrix_t<T> ref_line = reference_line_.cast<T>();
    Line<T, 2> boost_line(ref_line);

    T dist = DistanceTrajetory<T>(ref_line, trajectory);
    cost += T(params_->get<double>("weight_distance", 0.1)) * dist * dist;

    // Calculate jerk
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.2)));
    cost += T(params_->get<double>("weight_jerk", 2500.0)) * jerk * jerk;

    // Normalize
    residuals[0] = cost / (
      T(params_->get<double>("weight_distance", 0.1)) +
      T(params_->get<double>("weight_jerk", 2500.0)));
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

 private:
  Matrix_t<double> reference_line_;
  Matrix_t<double> initial_state_;
};

}  // namespace optimizer
