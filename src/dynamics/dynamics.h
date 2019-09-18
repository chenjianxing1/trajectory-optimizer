// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>

//! available dynamic models
#include "src/dynamics/state.h"
#include "src/dynamics/integration/euler.h"
#include "src/dynamics/integration/rk4.h"
#include "src/dynamics/models/single_track.h"
#include "src/commons/parameters.h"

namespace dynamics {
  using commons::Parameters;
  using geometry::Matrix_t;


  enum DynamicModels {
    SINGLE_TRACK = 0
  };

  template<typename T,
           Matrix_t<T> (*Fn)(const Matrix_t<T>&,
                             const Matrix_t<T>&,
                             const Parameters&)>
  Matrix_t<T> GenerateDynamicTrajectory(const Matrix_t<T>& initial_state,
                                        const Matrix_t<T>& input_vector,
                                        const Parameters& params) {
    Matrix_t<T> trajectory(input_vector.rows(),
                           initial_state.cols());
    trajectory.row(0) = initial_state;
    for (int i = 1; i < input_vector.rows(); i++) {
      trajectory.row(i) = Fn(trajectory.row(i-1),
                             input_vector.row(i-1),
                             params);
    }
    return trajectory;
  }

  template<typename T,
           Matrix_t<T> (*Fn)(const Matrix_t<T>&,
                             const Matrix_t<T>&,
                             const Parameters&)>
  Matrix_t<T> InputToTrajectory(const Matrix_t<T>& initial_state,
                                const Matrix_t<T>& input_vector,
                                const Parameters& params) {
    return input_vector;
  }
}  // namespace dynamics
