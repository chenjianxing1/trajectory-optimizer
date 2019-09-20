
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
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Distance;
using geometry::Line;
using geometry::Point;
using commons::Parameters;
using commons::CalculateJerk;
using optimizer::BaseFunctor;


class FollowReference : public BaseFunctor {
 public:
  FollowReference() :
    BaseFunctor(nullptr),
    reference_line_(),
    initial_state_() {}
  explicit FollowReference(Matrix_t<double> initial_state,
                           Parameters* params) :
    BaseFunctor(params),
    reference_line_(),
    initial_state_(initial_state) {}
  virtual ~FollowReference() {}

  template<typename T>
  T CalculateDistance(const Line<T, 2>& line, const Matrix_t<T>& trajectory) {
    T dist = T(0.);
    for ( int i = 1; i < trajectory.rows(); i++ ) {
      Point<T, 2> pt(T(trajectory(i, 0)), T(trajectory(i, 1)));
      dist += Distance<T, Line<T, 2>, Point<T, 2>>(line, pt);
    }
    return dist;
  }

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals, T cost = T(0.0)) {
    //! convert parameters to Eigen Matrix
    Matrix_t<T> trajectory = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    // calculate distance
    Line<T, 2> ref_line(reference_line_.cast<T>());
    T dist = CalculateDistance<T, 2>(ref_line, trajectory);
    cost += T(params_->get<double>("weight_distance", 0.1)) * dist * dist;

    // calculate jerk
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));
    cost += T(params_->get<double>("weight_jerk", 0.1)) * jerk * jerk;

    // TODO(@hart): normalize
    residuals[0] = cost;
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

  // TODO(@hart): could be a trajectory
  Matrix_t<double> reference_line_;
  Matrix_t<double> initial_state_;
};

}  // namespace optimizer
