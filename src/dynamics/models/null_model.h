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
using commons::ParameterPtr;
using commons::Parameter;


class NullModelStateDefinition : public StateDefinition {
 public:
  NullModelStateDefinition() {}
  ~NullModelStateDefinition() {}
  virtual int x() const { return -1; }
  virtual int y() const { return -1; }
  virtual int z() const { return -1; }
  virtual int vx() const { return -1; }
  virtual int vy() const { return -1; }
  virtual int vz() const { return -1; }
};


/**
 * @brief A model that does nothing
 * 
 */
class NullModel {
 public:
  NullModel() :
    state_def_(std::make_unique<NullModelStateDefinition>()) {}

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameter* params) {
    return u;
  }

  std::unique_ptr<StateDefinition> state_def_;
};

}  // namespace dynamics
