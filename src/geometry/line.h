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
#include "src/geometry/commons.h"

namespace geometry {

template <typename T, int N>
class Line : public BaseGeometry<Linestring_t<T, N>> {
 public:
  explicit Line(string color) : BaseGeometry<Linestring_t<T, N>>(color) {}
  Line() : BaseGeometry<Linestring_t<T, N>>() {}
  ~Line() {}
  void Append(Point<T, N> p) {
    bg::append(this->obj_, p.obj_);
  }
  explicit Line(const Matrix_t<T>& m) {
    for (int i = 0; i < m.rows(); i++)
      bg::append(this->obj_,
                 Point<T, N>(this->obj_.row(i)).obj_);
  }
  Matrix_t<T> toMatrix() const {
    Matrix_t<T> ret(this->obj_.size(), N);
    for (uint32_t i = 0; i < this->obj_.size(); i++) {
      ret.row(i) << getMatrix<T, N>(this->obj_[i]);
    }
    return ret;
  }
};

}  // namespace geometry
