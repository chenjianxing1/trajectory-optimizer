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

namespace dynamics {

using geometry::Matrix_t;

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

struct SingleTrackState : public DynamicState {
  explicit SingleTrackState(Matrix_t<double>* s) : DynamicState(s) {}
  ~SingleTrackState() {}
  double x() const { return (*state)(StateSingleTrackModel::X); }
  double y() const { return (*state)(StateSingleTrackModel::Y); }
  double z() const { return 0.0; }
  double v() const { return (*state)(StateSingleTrackModel::VELOCITY); }
  double yaw() const { return (*state)(StateSingleTrackModel::THETA); }
  double pitch() const { return 0.0; }
  double roll() const { return 0.0; }
};

template<typename T>
Matrix_t<T> fDotSingleTrack(const Matrix_t<T>& state,
                            const Matrix_t<T>& u,
                            T wheel_base = T(2.5)) {
  Matrix_t<T> A(4, 1);
  A << state(VELOCITY) * cos(state(THETA)),
       state(VELOCITY) * sin(state(THETA)),
       state(VELOCITY) * tan(u(STEERING_ANGLE)) / wheel_base,
       u(ACCELERATION);
  return A;
}

//! Fn is an integration function that returns the integrated state
template<typename T,
         Matrix_t<T> (*Fn)(const Matrix_t<T>&,
                           std::function<Matrix_t<T>(const Matrix_t<T>&)>,
                           T)>
Matrix_t<T> singleTrackModel(const DynamicState* state,
                             const Matrix_t<T>& u,
                             T dt,
                             T wheel_base = T(2.5)) {
  std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot =
    std::bind(fDotSingleTrack<T>,
              std::placeholders::_1,
              u,
              wheel_base);
  return Fn(*state->state, fDot, dt);
}

}  // namespace dynamics
