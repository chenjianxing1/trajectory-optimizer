
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
  bool operator()(T const* const* parameters, T* residuals, T cost = T(0.0)) {
    //! convert parameters to Eigen Matrix
    Matrix_t<T> trajectory = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    //! use boost ref line
    Line<T, 2> boost_line(reference_line_.cast<T>());
    Point<T, 2> pt(T(0.), T(1.0));
    std::cout << Distance(boost_line, pt);

    //! calculate jerk
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));

    cost += T(params_->get<double>("weight_jerk", 0.1)) * jerk * jerk;
    residuals[0] = cost;
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

  Matrix_t<double> reference_line_;
  Matrix_t<double> initial_state_;
};

}  // namespace optimizer
