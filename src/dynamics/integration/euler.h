// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"

namespace dynamics {
  template<typename T>
  Matrix_t<T>
  integrationEuler(const Matrix_t<T>& state,
                   std::function<Matrix_t<T>(const Matrix_t<T>&)> fDot,
                   T dt) {
    return state + dt * fDot(state);
  }
}  // namespace dynamics
