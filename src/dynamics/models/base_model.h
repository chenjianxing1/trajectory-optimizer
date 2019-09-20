
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
#include "src/dynamics/integration/rk4.h"

namespace dynamics {

using commons::Parameters;

class BaseModel {
 public:
  explicit BaseModel(Parameters* params) : params_(params) {}
  virtual ~BaseModel() = default;
  Parameters* params_;
};

}  // namespace dynamics
