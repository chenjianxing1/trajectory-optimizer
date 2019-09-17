// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "src/geometry/base.h"

namespace geometry {

template <typename T, int N>
class Point : public BaseGeometry<Point_t<T, N>> {
 public:
  explicit Point(string color) : BaseGeometry<Point_t<T, N>>(color) {}
  Point() : BaseGeometry<Point_t<T, N>>() {}

  ~Point() {}

  Point(T x, T y) {
    assert(N == 2);
    bg::set<0>(this->obj_, x);
    bg::set<1>(this->obj_, y);
  }

  Point(T x, T y, T z) {
    assert(N == 3);
    bg::set<0>(this->obj_, x);
    bg::set<1>(this->obj_, y);
    bg::set<2>(this->obj_, z);
  }

  explicit Point(const Matrix_t<T>& m) {
    bg::set<0>(this->obj_, m(0));
    bg::set<1>(this->obj_, m(1));
    if (N == 3)
      bg::set<2>(this->obj_, m(2));
  }

  // TODO(@hart): mult. dimensions
  Matrix_t<T> toMatrix() const {
    Matrix_t<T> matrix(1, N);
    matrix.row(0) << bg::get<0>(this->obj_),
                     bg::get<1>(this->obj_);
    return matrix;
  }
};

}  // namespace geometry
