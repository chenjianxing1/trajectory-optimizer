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
#include "src/geometry/line.h"

namespace geometry {

/**
 * @brief Implements a polygon class
 * 
 * @tparam T Type of data
 * @tparam N Dimensionality
 */
template <typename T, int N>
class Polygon : public BaseGeometry<Polygon_t<T, N>> {
 public:
  explicit Polygon(string color) : BaseGeometry<Polygon_t<T, N>>(color) {}
  Polygon() : BaseGeometry<Polygon_t<T, N>>() {}

  ~Polygon() {}

  void Append(Point<T, N> p) {
    bg::append(this->obj_, p.obj_);
  }

  explicit Polygon(const Matrix_t<T>& m) {
    for (int i = 0; i < m.rows(); i++)
      bg::append(this->obj_,
                 Point<T, N>(m(i, 0), m(i, 1)).obj_);
  }

  Matrix_t<T> toMatrix() const {
    Matrix_t<T> ret(bg::num_points(this->obj_), N);
    int i = 0;
    for (auto it = boost::begin(exterior_ring(this->obj_));
         it != boost::end(exterior_ring(this->obj_)); ++it) {
      ret.row(i) << getMatrix<T, N>(*it);
      i++;
    }
    return ret;
  }

  Line<T, N> ToLine() const {
    Line<T, N> ret_line;
    for (auto it = boost::begin(exterior_ring(this->obj_));
         it != boost::end(exterior_ring(this->obj_)); ++it) {
      bg::append(ret_line.obj_, *it);
    }
    return ret_line;
  }
};


}  // namespace geometry
