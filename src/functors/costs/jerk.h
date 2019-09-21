
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameters;
using commons::CalculateJerk;

class JerkCost : public BaseCost {
 public:
  JerkCost() : BaseCost(nullptr) {}
  explicit JerkCost(Parameters* params) :
    BaseCost(params) {}
  virtual ~JerkCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) {
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));
    return T(params_->get<double>("weight_jerk", 1000.0)) * jerk * jerk;
  }
};

}  // namespace optimizer
