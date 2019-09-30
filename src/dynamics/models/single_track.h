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
#include "src/dynamics/state.h"

namespace dynamics {

using geometry::Matrix_t;
using commons::ParameterPtr;
using commons::Parameter;
using dynamics::StateDefinition;

class SingleTrackStateDefinition : public StateDefinition {
 public:
  SingleTrackStateDefinition() {}
  ~SingleTrackStateDefinition() {}
  virtual int x() const { return 0; }
  virtual int y() const { return 1; }
  virtual int z() const { return -1; }
  virtual int v() const { return 3; }
  virtual int theta() const { return 2; }
};


/**
 * @brief Simplified single-track model
 * 
 */
class SingleTrackModel {
 public:
  SingleTrackModel() :
    state_def_(std::make_unique<SingleTrackStateDefinition>()) {}
  ~SingleTrackModel() {}

  enum StateDefinitionS {
    X = 0,
    Y = 1,
    THETA = 2,
    VELOCITY = 3
  };

  enum InputDefinitionS {
    STEERING_ANGLE = 0,
    ACCELERATION = 1
  };

  template<typename T>
  static Matrix_t<T> fDot(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          const T& wheel_base) {
    Matrix_t<T> A(1, state.cols());
    A << state(VELOCITY) * cos(state(THETA)),
         state(VELOCITY) * sin(state(THETA)),
         state(VELOCITY) * tan(u(STEERING_ANGLE)) / wheel_base,
         u(ACCELERATION);
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

  std::unique_ptr<StateDefinition> state_def_;
};

}  // namespace dynamics
