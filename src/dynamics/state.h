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

struct DynamicState {
  explicit DynamicState(Matrix_t<double>* s) : state(s) {}
  virtual ~DynamicState() = default;
  virtual double x() const = 0;
  virtual double y() const = 0;
  virtual double z() const = 0;
  virtual double v() const = 0;
  virtual double yaw() const = 0;
  virtual double pitch() const = 0;
  virtual double roll() const = 0;
  /*
  virtual double qx() const = 0;
  virtual double qy() const = 0;
  virtual double qz() const = 0;
  virtual double qw() const = 0;
  */
  Matrix_t<double>* state;
};

}  // namespace dynamics
