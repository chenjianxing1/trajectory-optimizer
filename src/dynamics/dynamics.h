// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>

//! available dynamic models
#include "src/dynamics/integration/euler.h"
#include "src/dynamics/integration/rk4.h"
#include "src/dynamics/models/single_track.h"
#include "src/dynamics/models/null_model.h"
#include "src/commons/parameters.h"

namespace dynamics {
  using commons::ParameterPtr;
  using commons::Parameter;
  using geometry::Matrix_t;

  enum DynamicModels {
    SINGLE_TRACK = 0,
    COPY_MODEL = 1
  };

  template<typename T, class M, class I>
  inline Matrix_t<T> GenerateDynamicTrajectory(
    const Matrix_t<T>& initial_states,
    const Matrix_t<T>& input_vector,
    Parameter* params) {
    Matrix_t<T> trajectory(input_vector.rows(),
                           initial_states.cols());
    trajectory.block(0,
                     0,
                     initial_states.rows(),
                     initial_states.cols()) = initial_states;
    for (int i = initial_states.rows() - 1; i < input_vector.rows(); i++) {
      trajectory.row(i) = M::template Step<T, I>(
        trajectory.row(i-1),
        input_vector.row(i-1),
        params);
    }
    return trajectory;
  }

}  // namespace dynamics
