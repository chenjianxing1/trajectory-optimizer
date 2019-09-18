#pragma once
#include <limits>
#include <utility>
#include "src/geometry/geometry.h"

namespace commons {
using geometry::Matrix_t;

template<typename T>
Matrix_t<T> CalculateDiff(const Matrix_t<T>& traj, T dt) {
  Matrix_t<T> ret_traj(traj.rows() - 1, traj.cols());
  for (int i = 1; i < traj.rows(); i++) {
    ret_traj.row(i-1) = (traj.row(i) - traj.row(i-1)) / dt;
  }
  return ret_traj;
}

template<typename T>
T CalculateJerk(const Matrix_t<T>& traj, T dt) {
  Matrix_t<T> reduced_traj = traj.block(0, 0, traj.rows(), 2);
  Matrix_t<T> traj_v = CalculateDiff(reduced_traj, dt);
  Matrix_t<T> traj_a = CalculateDiff(traj_v, dt);
  Matrix_t<T> traj_j = CalculateDiff(traj_a, dt);
  return traj_j.sum();
}


}  // namespace commons
