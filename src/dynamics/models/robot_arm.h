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
class RobotArm {
 public:
  RobotArm() {}
  ~RobotArm() {}

  enum class StateDefinition {
    X = 0,
    Y = 1,
    THETA = -1,
    VELOCITY = -1,
    Z = -1
  };

  enum class InputDefinition {
    STEERING_ANGLE = -1,
    ACCELERATION = -1
  };

  template<typename T>
  static Matrix_t<T> fDot(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameter* params) {
    Matrix_t<T> ret_state(1, state.cols());
    T x0 = T(params->get<double>("x0", .0));
    T x1 = T(params->get<double>("x1", .0));
    T l0 = T(params->get<double>("l0", .5));
    T l1 = T(params->get<double>("l1", .5));
    ret_state << l0*cos(u(0)) + x0 + l1*cos(u(1)),
                 l0*sin(u(0)) + x1 + l1*sin(u(1));
    return ret_state;
  }

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameter* params) {
    // std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot_ =
    //   std::bind(fDot<T>,
    //             std::placeholders::_1,
    //             u,
    //             params);
    // return I::template Integrate<T>(state,
    //                                 fDot_,
    //                                 T(params->get<double>("dt", 0.1)));
    return fDot<T>(state, u, params);
  }

};

}  // namespace dynamics
