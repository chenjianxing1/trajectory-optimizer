// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/dynamics/state.h"
#include "src/dynamics/dynamics.h"
#include "src/commons/parameters.h"

namespace dynamics {

using geometry::Matrix_t;
using parameters::Parameters;

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


template<typename T>
Matrix_t<T> fDotSingleTrack(const Matrix_t<T>& state,
                            const Matrix_t<T>& u,
                            const Parameters& params) {
  Matrix_t<T> A(4, 1);
  A << state(VELOCITY) * cos(state(THETA)),
       state(VELOCITY) * sin(state(THETA)),
       state(VELOCITY) * tan(u(STEERING_ANGLE)) / params.get<T>("wheel_base", T(2.7)),
       u(ACCELERATION);
  return A;
}

//! Fn is an integration function that returns the integrated state
template<typename T,
         Matrix_t<T> (*Fn)(const Matrix_t<T>&,
                           std::function<Matrix_t<T>(const Matrix_t<T>&)>,
                           T)>
Matrix_t<T> singleTrackModel(const Matrix_t<T>& state,
                             const Matrix_t<T>& u,
                             const Parameters& params) {
  std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot =
    std::bind(fDotSingleTrack<T>,
              std::placeholders::_1,
              u,
              params);
  return Fn(state, fDot, params.get<T>("dt", T(0.1)));
}

}  // namespace dynamics
