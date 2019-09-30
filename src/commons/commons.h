// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <limits>
#include <utility>
#include <vector>
#include <ceres/ceres.h>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"

namespace commons {

using geometry::Matrix_t;
using geometry::Line;
using geometry::Point;
using geometry::Polygon;
using geometry::Distance;
using commons::Parameter;
using commons::ParameterPtr;

typedef std::pair<double, Matrix_t<double>> TimedPolygonOutline;

inline Matrix_t<double> InterpolateMatrices(const Matrix_t<double>& p0,
                                            const Matrix_t<double>& p1,
                                            double lambda) {
  return (1. - lambda)*p0 + lambda*p1;
}

inline Matrix_t<double> InterpolateTimedPolygon(const TimedPolygonOutline& p0,
                                                const TimedPolygonOutline& p1,
                                                double time) {
  double lambda = (time - p0.first) / (p1.first - p0.first);
  return InterpolateMatrices(p0.second, p1.second, lambda);
}

// TODO(@hart): write test
class ObjectOutline {
 public:
  ObjectOutline() {}

  ObjectOutline(const Matrix_t<double>& outline,
                double timestamp = 0) {
    this->Add(outline, timestamp);
  }

  void Add(const Matrix_t<double>& outline, double timestamp = 0) {
    TimedPolygonOutline timed_outline = std::make_pair(timestamp, outline);
    object_outlines_.push_back(timed_outline);
  }

  Matrix_t<double> Query(double timestamp_query) const {
    for (int i = 0; i < object_outlines_.size() - 1; i++) {
      if (object_outlines_.at(i).first <= timestamp_query && \
          object_outlines_.at(i+1).first > timestamp_query) {
        return InterpolateTimedPolygon(object_outlines_.at(i),
                                       object_outlines_.at(i+1),
                                       timestamp_query);
      }
    }
    if (timestamp_query >= object_outlines_.back().first)
      return object_outlines_.back().second;
    return object_outlines_.front().second;
  }

 private:
  std::vector<TimedPolygonOutline> object_outlines_;
};


template<typename T>
inline Matrix_t<T> CalculateDiff(const Matrix_t<T>& traj, const T& dt) {
  Matrix_t<T> ret_traj(traj.rows() - 1, traj.cols());
  for (int i = 1; i < traj.rows(); i++) {
    ret_traj.row(i-1) = (traj.row(i) - traj.row(i-1)) / dt;
  }
  return ret_traj;
}

template<typename T, class M>
inline T CalculateSquaredJerk(const Matrix_t<T>& traj, const T& dt) {
  // TODO(@hart): make more efficient
  Matrix_t<T> reduced_traj(traj.rows(), 2);
  if (static_cast<int>(M::StateDefinition::Z) != -1) {
    reduced_traj.resize(traj.rows(), 3);
  }
  reduced_traj.col(0) = traj.col(static_cast<int>(M::StateDefinition::X));
  reduced_traj.col(1) = traj.col(static_cast<int>(M::StateDefinition::Y));
  if (static_cast<int>(M::StateDefinition::Z) != -1) {
    reduced_traj.col(2) = traj.col(static_cast<int>(M::StateDefinition::Z));
  }
  Matrix_t<T> traj_v = CalculateDiff(reduced_traj, dt);
  Matrix_t<T> traj_a = CalculateDiff(traj_v, dt);
  Matrix_t<T> traj_j = CalculateDiff(traj_a, dt);
  T jerk = T(0.);
  for (int i = 0; i < traj_j.rows(); i++) {
    for (int j = 0; j < traj_j.cols(); j++) {
      jerk += traj_j(i, j)*traj_j(i, j);
    }
  }
  return jerk;
}

template<typename T, class M>
inline T CalculateSquaredDistance(const Line<T, 2>& line,
                                  const Matrix_t<T>& trajectory,
                                  T dist = T(0.)) {
  T tmp_dist = T(0.);
  Point<T, 2> pt;
  for ( int i = 0; i < trajectory.rows(); i++ ) {
    // TODO(@hart): use state definitions
    boost::geometry::set<0>(pt.obj_,
      trajectory(i, static_cast<int>(M::StateDefinition::X)));
    boost::geometry::set<1>(pt.obj_,
      trajectory(i, static_cast<int>(M::StateDefinition::Y)));
    tmp_dist = Distance<T, Line<T, 2>, Point<T, 2>>(line, pt);
    dist += tmp_dist*tmp_dist;
  }
  return dist;
}

template<typename T, class M>
inline T GetSquaredObjectCosts(const ObjectOutline& obj_out,
                               const Matrix_t<T>& trajectory,
                               const T& epsilon,
                               double dt) {
  T tmp_dist = T(0.);
  T dist = T(0.);
  Point<T, 2> pt;
  Polygon<T, 2> poly;
  for ( int i = 0; i < trajectory.rows(); i++ ) {
    Matrix_t<double> object_outline =
      obj_out.Query(i*dt);
    // TODO(@hart): use state definitions
    boost::geometry::set<0>(pt.obj_,
      trajectory(i, static_cast<int>(M::StateDefinition::X)));
    boost::geometry::set<1>(pt.obj_,
      trajectory(i, static_cast<int>(M::StateDefinition::Y)));
    poly = Polygon<T, 2>(object_outline.cast<T>());
    tmp_dist = Distance<T, 2>(poly, pt);
    if (tmp_dist < epsilon) {
      dist += (epsilon - tmp_dist)*(epsilon - tmp_dist);
    }
  }
  return dist;
}

template<typename T, class M>
inline T CalculateSquaredDistance(const Matrix_t<T>& traj0,
                                  const Matrix_t<T>& traj1,
                                  T dist = T(0.)) {
  T tmp_dist = T(0.);
  T loc = T(0.);
  for (int i = 0; i < traj0.rows(); i++) {
    loc = T(0.);
    for (int j = 0; j < traj0.cols(); j++) {
      loc += (traj0(i, j) - traj1(i, j))*(traj0(i, j) - traj1(i, j));
    }
    tmp_dist = loc;
    dist += tmp_dist*tmp_dist;
  }
  return dist;
}

}  // namespace commons

