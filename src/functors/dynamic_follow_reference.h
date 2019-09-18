
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameters;
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
  bool operator()(T const* const* parameters, T* residuals) {
    T cost = T(0.0);
    //! convert parameters to Eigen Matrix
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    // either (x, y) or (x, y, theta, v)
    Matrix_t<T> trajectory =
      GenerateDynamicTrajectory<T, SingleTrackModel<T, integrationRK4>>(
        initial_state_t, opt_vec, *this->GetParams());

    //! debug
    T dist = T(0.0);
    for ( int i = 1; i < trajectory.rows(); i++ ) {
      dist += ceres::sqrt((trajectory(i, 1) - T(2.0))*(trajectory(i, 1) - T(2.0)));
    }
    cost += T(this->GetParams()->get<double>("weight_distance", 0.1)) * dist * dist;

    //! calculate jerk
    T jerk = CalculateJerk<T>(
      trajectory,
      T(this->GetParams()->get<double>("dt", 0.1)));

    cost += T(this->GetParams()->get<double>("weight_jerk", 100.0)) * jerk * jerk;
    std::cout << opt_vec << std::endl;
    residuals[0] = cost / (
      T(this->GetParams()->get<double>("weight_distance", 0.1)) +
      T(this->GetParams()->get<double>("weight_jerk", 100.0)));
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
