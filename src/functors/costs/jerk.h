
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include <memory>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameter;
using commons::CalculateSquaredJerk;
using commons::ParameterPtr;


class JerkCost : public BaseCost {
 public:
  JerkCost() : BaseCost() {}
  explicit JerkCost(const ParameterPtr& params,
                    double cost = 10.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_jerk", cost);
  }
  virtual ~JerkCost() {}

  template<typename T, class M>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) const {
    std::cout << int(M::StateDefinition::Z) << std::endl;
    T jerk = CalculateSquaredJerk<T, M>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));
    return Weight<T>() * jerk;
  }

};

typedef std::shared_ptr<JerkCost> JerkCostPtr;

}  // namespace optimizer
