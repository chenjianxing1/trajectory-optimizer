// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include "src/geometry/geometry.h"
#include "src/dynamics/integration/base_integration.h"

namespace dynamics {

class IntegrationRK4 : public BaseIntegration {
 public:
  IntegrationRK4() : BaseIntegration() {}
  virtual ~IntegrationRK4() {}

  template<typename T>
  static Matrix_t<T> Integrate(
    const Matrix_t<T>& state,
    std::function<Matrix_t<T>(const Matrix_t<T>&)> fDot,
    T dt) {
    Matrix_t<T> k0 = dt*fDot(state);
    Matrix_t<T> k1 = dt*fDot(state + k0/T(2.0));
    Matrix_t<T> k2 = dt*fDot(state + k1/T(2.0));
    Matrix_t<T> k3 = dt*fDot(state + k2);
    return state + T(1.0/6.0)*(k0 + T(2.0)*k1 + T(2.0)*k2 + k3);
  }
};

}  // namespace dynamics
