// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/dynamics/dynamics.h"
#include "src/dynamics/integration/rk4.h"
#include "src/dynamics/integration/euler.h"

namespace dynamics {

using geometry::Matrix_t;
using commons::ParameterPtr;
using commons::Parameter;

/**
 * @brief Simplified single-track model
 * 
 */
class SingleTrackModel {
 public:
  SingleTrackModel() {}
  ~SingleTrackModel() {}

  enum class StateDefinition {
    X = 0,
    Y = 1,
    THETA = 2,
    VELOCITY = 3,
    Z = -1
  };

  enum class InputDefinition {
    STEERING_ANGLE = 0,
    ACCELERATION = 1
  };

  template<typename T>
  static Matrix_t<T> fDot(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          const T& wheel_base) {
    Matrix_t<T> A(1, state.cols());
    A << state(static_cast<int>(StateDefinition::VELOCITY)) * \
         cos(state(static_cast<int>(StateDefinition::THETA))),
         state(static_cast<int>(StateDefinition::VELOCITY)) * \
         sin(state(static_cast<int>(StateDefinition::THETA))),
         state(static_cast<int>(StateDefinition::VELOCITY)) * \
         tan(u(static_cast<int>(InputDefinition::STEERING_ANGLE))) / wheel_base,
         u(static_cast<int>(InputDefinition::ACCELERATION));
    return A;
  }

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameter* params) {
    std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot_ =
      std::bind(fDot<T>,
                std::placeholders::_1,
                u,
                T(params->get<double>("wheel_base", 2.7)));
    return I::template Integrate<T>(state,
                                    fDot_,
                                    T(params->get<double>("dt", 0.1)));
  }

};

}  // namespace dynamics
