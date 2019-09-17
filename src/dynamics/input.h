// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include "src/geometry/geometry.h"
#include "src/dynamics/dynamics.h"

namespace dynamics {

using geometry::Matrix_t;

struct InputState {
  explicit InputState(Matrix_t<double>* s) : state(s) {}
  virtual ~InputState() = default;
  Matrix_t<double>* state;
};

}  // namespace dynamics
