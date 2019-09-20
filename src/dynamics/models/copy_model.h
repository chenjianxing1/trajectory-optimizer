// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/dynamics/dynamics.h"

namespace dynamics {

using geometry::Matrix_t;
using commons::Parameters;

class CopyModel {
 public:
  explicit CopyModel(Parameters* params) {}

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameters* params) {
    return u;
  }
};

}  // namespace dynamics
