
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <memory>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameter;
using commons::ParameterPtr;

class InputCost : public BaseCost {
 public:
  InputCost() : BaseCost() {}
  explicit InputCost(const ParameterPtr& params,
                     double cost = 200.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_input", cost);
  }
  virtual ~InputCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T cost = T(0.)) const {
    for (int i = 0; i < inputs.cols(); i++) {
      // check if within bounds
      for (int j = 0; j < inputs.rows(); j++) {
        if (inputs(j, i) < lower_bounds_(0, i))
          cost += ceres::pow(inputs(j, i) - lower_bounds_(0, i), 2);
        if (inputs(j, i) > upper_bounds_(0, i))
          cost += ceres::pow(inputs(j, i) - upper_bounds_(0, i), 2);
      }
    }
    return Weight<T>() * cost;
  }

  void SetLowerBound(const Matrix_t<double>& lb) {
    lower_bounds_ = lb;
  }

  void SetUpperBound(const Matrix_t<double>& ub) {
    upper_bounds_ = ub;
  }

  Matrix_t<double> lower_bounds_;
  Matrix_t<double> upper_bounds_;
};

typedef std::shared_ptr<InputCost> InputCostPtr;

}  // namespace optimizer
