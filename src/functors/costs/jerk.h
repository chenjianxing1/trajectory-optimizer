
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
using commons::Parameter;
using commons::CalculateJerk;
using commons::ParameterPtr;


class JerkCost : public BaseCost {
 public:
  JerkCost() : BaseCost() {}
  explicit JerkCost(const ParameterPtr& params) :
    BaseCost(params) {
      weight_ = params_->get<double>("weight_jerk", 100.);
  }
  virtual ~JerkCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) const {
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));
    return Weight<T>() * jerk;
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }
};

typedef std::shared_ptr<JerkCost> JerkCostPtr;

}  // namespace optimizer
