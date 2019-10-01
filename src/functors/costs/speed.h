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
using geometry::BaseGeometry;
using geometry::Line;
using commons::ParameterPtr;
using commons::Parameter;
using commons::CalculateSquaredDistance;

/**
 * @brief Cost for deviation to desired speed
 * 
 */
class SpeedCost : public BaseCost {
 public:
  SpeedCost() : BaseCost(), v_des_() {}
  explicit SpeedCost(const ParameterPtr& params,
                     double cost = 100.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_speed", cost);
  }
  virtual ~SpeedCost() {}

  template<typename T, class M>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T cost = T(0.)) const {
    for (int i = 0; i < trajectory.rows(); i++) {
      T v_total = T(0.);
      if (static_cast<int>(M::StateDefinition::VELOCITY) != -1) {
        int vel_idx = static_cast<int>(M::StateDefinition::VELOCITY);
        v_total = trajectory(i, vel_idx);
      } else {
        int vel_idx_x = static_cast<int>(M::StateDefinition::VX);
        int vel_idx_y = static_cast<int>(M::StateDefinition::VY);
        int vel_idx_z = static_cast<int>(M::StateDefinition::VZ);
        v_total = ceres::sqrt(
          trajectory(i, vel_idx_x)*trajectory(i, vel_idx_x) + \
          trajectory(i, vel_idx_y)*trajectory(i, vel_idx_y) + \
          trajectory(i, vel_idx_z)*trajectory(i, vel_idx_z));
      }
      cost += (v_total - v_des_)*(v_total - v_des_);
    }
    return cost;
  }

  void SetDesiredSpeed(double speed) {
    v_des_ = speed;
  }

  double v_des_;
};

typedef std::shared_ptr<SpeedCost> SpeedCostPtr;

}  // namespace optimizer
