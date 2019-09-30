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
#include "src/dynamics/models/triple_int.h"
#include "src/commons/parameters.h"

namespace dynamics {
  using commons::ParameterPtr;
  using commons::Parameter;
  using geometry::Matrix_t;

  enum DynamicModels {
    SINGLE_TRACK = 0,
    COPY_MODEL = 1,
    TRIPLE_INT = 1,
  };
  
  /**
   * @brief Function that generates a dynamic trajectory
   * 
   * @tparam T Type of data
   * @tparam M Dynamic model used
   * @tparam I Integration method (euler, rk4, ..)
   * @param initial_states Initial state(s) for trajectory 
   * @param input_vector Input vector of size (N, InputSize)
   * @param params Parameters, such as delta time, wheel_base etc.
   * @return Matrix_t<T> Trajectory of size (N, State)
   */
  template<typename T, class M, class I>
  inline Matrix_t<T> GenerateDynamicTrajectory(
    const Matrix_t<T>& initial_states,
    const Matrix_t<T>& input_vector,
    Parameter* params) {
    int total_rows = input_vector.rows() + initial_states.rows();
    Matrix_t<T> trajectory(total_rows,
                           initial_states.cols());
    trajectory.block(0,
                     0,
                     initial_states.rows(),
                     initial_states.cols()) = initial_states;
    // assume its correct up to here
    int count = 0;
    for (int i = initial_states.rows(); i < total_rows; i++) {
      trajectory.row(i) = M::template Step<T, I>(
        trajectory.row(i-1),
        input_vector.row(count),
        params);
      count++;
    }
    return trajectory;
  }

}  // namespace dynamics
