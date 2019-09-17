// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "src/geometry/base.h"

namespace geometry {

namespace bg = boost::geometry;
using boost::begin;
using boost::end;

//! Point to Eigen
template <typename T, int N>
Matrix_t<T> getMatrix(const Point_t<T, N> &p) {
  Matrix_t<T> ret(1, N);
  ret.row(0) << bg::get<0>(p), bg::get<1>(p);
  return ret;
}

//! Line to Eigen
template <typename T, int N>
Matrix_t<T> getMatrix(const Linestring_t<T, N> &l) {
  Matrix_t<T> ret(l.size(), N);
  for (uint32_t i = 0; i < l.size(); i++) {
    ret.row(i) << getMatrix<T, N>(l[i]);
  }
  return ret;
}

//! Shape to Eigen
template <typename T, int N>
Matrix_t<T> getMatrix(const Polygon_t<T, N> &p) {
  Matrix_t<T> ret(bg::num_points(p), N);
  int i = 0;
  for (auto it = boost::begin(exterior_ring(p));
      it != boost::end(exterior_ring(p)); ++it) {
    ret.row(i) << getMatrix<T, N>(*it);
    i++;
  }
  return ret;
}

// TODO(@hart): add line and bezier functionality

}  // namespace geometry
