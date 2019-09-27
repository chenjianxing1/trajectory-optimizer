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
using geometry::BaseGeometry;
using geometry::Line;
using commons::ParameterPtr;
using commons::Parameter;
using commons::CalculateDistance;

/**
 * @brief Cost for deviation to desired speed
 * 
 */
class SpeedCost : public BaseCost {
 public:
  SpeedCost() : BaseCost(), v_des_() {}
  explicit SpeedCost(const ParameterPtr& params) :
    BaseCost(params) {
      weight_ = params_->get<double>("weight_speed", 0.1);
  }
  virtual ~SpeedCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T cost = T(0.)) const {
    for (int i = 0; i < trajectory.rows(); i++) {
      cost += (trajectory(i, 3) - v_des_)*(trajectory(i, 3) - v_des_);
    }
    return cost;
  }

  void SetDesiredSpeed(double speed) {
    v_des_ = speed;
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  double v_des_;
};

typedef std::shared_ptr<SpeedCost> SpeedCostPtr;

}  // namespace optimizer
