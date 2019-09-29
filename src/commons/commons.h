// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <limits>
#include <ceres/ceres.h>
#include <utility>
#include "src/geometry/geometry.h"

namespace commons {
using geometry::Matrix_t;
using geometry::Line;
using geometry::Point;
using geometry::Polygon;
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
inline T CalculateSquaredJerk(const Matrix_t<T>& traj, T dt) {
  Matrix_t<T> reduced_traj = traj.block(0, 0, traj.rows(), 2);
  // TODO(@hart): make more efficient
  Matrix_t<T> traj_v = CalculateDiff(reduced_traj, dt);
  Matrix_t<T> traj_a = CalculateDiff(traj_v, dt);
  Matrix_t<T> traj_j = CalculateDiff(traj_a, dt);
  T jerk = T(0.);
  for (int i = 0; i < traj_j.rows(); i++) {
    jerk += traj_j(i, 0)*traj_j(i, 0);
    jerk += traj_j(i, 1)*traj_j(i, 1);
  }
  return jerk;
}

template<typename T>
inline T CalculateSquaredDistance(const Line<T, 2>& line,
                                  const Matrix_t<T>& trajectory,
                                  T dist = T(0.)) {
  T tmp_dist = T(0.);
  Point<T, 2> pt;
  for ( int i = 0; i < trajectory.rows(); i++ ) {
    boost::geometry::set<0>(pt.obj_, trajectory(i, 0));
    boost::geometry::set<1>(pt.obj_, trajectory(i, 1));
    tmp_dist = Distance<T, Line<T, 2>, Point<T, 2>>(line, pt);
    dist += tmp_dist*tmp_dist;
  }
  return dist;
}

template<typename T>
inline T GetSquaredObjectCosts(const Polygon<T, 2>& poly,
                               const Matrix_t<T>& trajectory,
                               T eps = T(1.)) {
  T tmp_dist = T(0.);
  T dist = T(0.);
  Point<T, 2> pt;
  for ( int i = 0; i < trajectory.rows(); i++ ) {
    boost::geometry::set<0>(pt.obj_, trajectory(i, 0));
    boost::geometry::set<1>(pt.obj_, trajectory(i, 1));
    tmp_dist = Distance<T, 2>(poly, pt);
    if (tmp_dist < T(eps)) {
      dist += (T(eps) - tmp_dist)*(T(eps) - tmp_dist);
    }
  }
  return dist;
}

template<typename T>
inline T CalculateSquaredDistance(const Matrix_t<T>& traj0,
                                  const Matrix_t<T>& traj1,
                                  T dist = T(0.)) {
  T tmp_dist = T(0.);
  for (int i = 0; i < traj0.rows(); i++) {
    T loc = T(0.);
    for (int j = 0; j < traj0.cols(); j++) {
      loc += (traj0(i, j) - traj1(i, j))*(traj0(i, j) - traj1(i, j));
    }
    tmp_dist = loc;
    dist += tmp_dist*tmp_dist;
  }
  return dist;
}

}  // namespace commons

