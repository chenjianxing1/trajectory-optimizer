// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include "src/geometry/geometry.h"

namespace dynamics {

using geometry::Matrix_t;

class StateDefinition {
 public:
  StateDefinition() {}
  virtual ~StateDefinition() = default;
  // return -1 if not implemented
  virtual int x() const {}
  virtual int y() const {}
  virtual int z() const {}
  virtual int v() const {}
  virtual int theta() const {}
  virtual int vx() const {}
  virtual int vy() const {}
  virtual int vz() const {}
};

}  // namespace dynamics
