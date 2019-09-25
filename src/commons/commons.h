// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <limits>
#include <utility>
#include "src/geometry/geometry.h"

namespace commons {
using geometry::Matrix_t;
using geometry::Line;
using geometry::Point;
using geometry::Distance;

template<typename T>
inline Matrix_t<T> CalculateDiff(const Matrix_t<T>& traj, T dt) {
  Matrix_t<T> ret_traj(traj.rows() - 1, traj.cols());
  for (int i = 1; i < traj.rows(); i++) {
    ret_traj.row(i-1) = (traj.row(i) - traj.row(i-1)) / dt;
  }
  return ret_traj;
}

template<typename T>
inline T CalculateJerk(const Matrix_t<T>& traj, T dt) {
  Matrix_t<T> reduced_traj = traj.block(0, 0, traj.rows(), 2);
  Matrix_t<T> traj_v = CalculateDiff(reduced_traj, dt);
  Matrix_t<T> traj_a = CalculateDiff(traj_v, dt);
  Matrix_t<T> traj_j = CalculateDiff(traj_a, dt);
  return traj_j.sum();
}

template<typename T>
inline T CalculateDistance(const Line<T, 2>& line,
                           const Matrix_t<T>& trajectory,
                           T dist = T(0.)) {
  for ( int i = 1; i < trajectory.rows(); i++ ) {
    Point<T, 2> pt(T(trajectory(i, 0)), T(trajectory(i, 1)));
    dist += Distance<T, Line<T, 2>, Point<T, 2>>(line, pt);
  }
  return dist;
}

}  // namespace parameters
