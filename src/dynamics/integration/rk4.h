// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include "src/geometry/geometry.h"

namespace dynamics {
  template<typename T>
  Matrix_t<T>
  integrationRK4(const Matrix_t<T>& state,
                 std::function<Matrix_t<T>(const Matrix_t<T>&)> fDot,
                 T dt) {
    Matrix_t<T> k0 = dt*fDot(state);
    Matrix_t<T> k1 = dt*fDot(state + k0/2);
    Matrix_t<T> k2 = dt*fDot(state + k1/2);
    Matrix_t<T> k3 = dt*fDot(state + k2);
    return state + 1.0/6.0*(k0 + 2*k1 + 2*k2 + k3);
  }
}  // namespace dynamics
