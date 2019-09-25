
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
using commons::ParameterPtr;

class InputCost : public BaseCost {
 public:
  InputCost() : BaseCost() {}
  explicit InputCost(const ParameterPtr& params) :
    BaseCost(params) {
      weight_ = params_->get<double>("weight_input", 100.0);
    }
  virtual ~InputCost() {}

  template<typename T>
  T SquaredNorm(const T& x, int N = 2) const {
    return ceres::sqrt(ceres::pow(x, N));
  }

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) const {
    T cost = T(0.);
    for (int i = 0; i < inputs.cols(); i++) {
      // check if within bounds
      for (int j = 0; j < inputs.rows(); j++) {
        if (inputs(j, i) < lower_bounds_(0, i))
          cost += SquaredNorm<T>(inputs(j, i) - lower_bounds_(0, i));
        if (inputs(j, i) > upper_bounds_(0, i))
          cost += SquaredNorm<T>(inputs(j, i) - upper_bounds_(0, i));
      }
    }
    return Weight<T>() * cost * cost;
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
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
