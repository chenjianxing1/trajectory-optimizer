// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>

//! available dynamic models
#include "src/dynamics/state.h"
#include "src/dynamics/integration/euler.h"
#include "src/dynamics/integration/rk4.h"
#include "src/dynamics/models/single_track.h"

namespace dynamics {
  enum DynamicModels {
    SINGLE_TRACK = 0
  };

}  // namespace dynamics
