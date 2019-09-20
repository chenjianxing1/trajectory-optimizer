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
#include "src/dynamics/models/base_model.h"

namespace dynamics {

using geometry::Matrix_t;
using commons::Parameters;

enum StateSingleTrackModel {
  X = 0,
  Y = 1,
  THETA = 2,
  VELOCITY = 3,
};

enum InputSingleTrackModel {
  STEERING_ANGLE = 0,
  ACCELERATION = 1,
};

class SingleTrackModel : public BaseModel {
 public:
  explicit SingleTrackModel(Parameters* params) :
    BaseModel(params) {}

  template<typename T>
  static Matrix_t<T> fDot(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameters* params) {
    Matrix_t<T> A(1, state.cols());
    T wheel_base = T(params->get<double>("wheel_base", 2.7));
    A << state(VELOCITY) * cos(state(THETA)),
         state(VELOCITY) * sin(state(THETA)),
         state(VELOCITY) * tan(u(STEERING_ANGLE)) / wheel_base,
         u(ACCELERATION);
    return A;
  }

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameters* params) {
    std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot_ =
      std::bind(fDot<T>,
                std::placeholders::_1,
                u,
                params);
    return I::template Integrate<T>(state,
                                    fDot_,
                                    T(params->get<double>("dt", 0.1)));
  }
};

}  // namespace dynamics
