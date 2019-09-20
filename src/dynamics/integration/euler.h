// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"

namespace dynamics {
using geometry::Matrix_t;

class IntegrationEuler {
 public:
  IntegrationEuler() {}
  template<typename T>
  static Matrix_t<T> Integrate(
    const Matrix_t<T>& state,
    std::function<Matrix_t<T>(const Matrix_t<T>&)> fDot,
    T dt) {
    return state + dt * fDot(state);
  }
};

}  // namespace dynamics
