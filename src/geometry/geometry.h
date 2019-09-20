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
#include "src/geometry/point.h"
#include "src/geometry/line.h"
#include "src/geometry/polygon.h"

namespace geometry {

namespace bg = boost::geometry;
using boost::begin;
using boost::end;

template <typename T, class A, class B>
T Distance(const A& a, const B& b) {
  return bg::distance(a.obj_, b.obj_);
}

template <class A, class B>
bool Collides(const A& a, const B& b) {
  return bg::intersects(a.obj_, b.obj_);
}

template <class A, class B>
bool Disjoint(const A& a, const B& b) {
  return bg::disjoint(a.obj_, b.obj_);
}

template <class A, class B>
bool Within(const A& a, const B& b) {
  return bg::within(a.obj_, b.obj_);
}

//! translate geometry
template <class A, class B>
A Translate(const A& geom,
            const B& p) {
  namespace trans = boost::geometry::strategy::transform;
  using boost::geometry::dsv;
  A translated_geometry;
  trans::translate_transformer<double, 2, 2> translate(bg::get<0>(p.obj_),
                                                       bg::get<1>(p.obj_));
  boost::geometry::transform(geom.obj_, translated_geometry.obj_, translate);
  return translated_geometry;
}

//! rotate geometry
template <class A>
A Rotate(const A& geom, double angle) {
  using  boost::geometry::strategy::transform::rotate_transformer;
  A geom_new;
  rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(-angle);
  boost::geometry::transform(geom.obj_, geom_new.obj_, rotate);
  return geom_new;
}

//! s functions
template <typename T>
inline Point<T, 3> GetPoseAtS(Line<T, 2> l, T s) {
  float length = 0.0f;
  for (uint32_t i = 0; i < l.obj_.size() - 1; i++) {
    float dx = (bg::get<0>(l.obj_[i + 1]) - bg::get<0>(l.obj_[i]));
    float dy = (bg::get<1>(l.obj_[i + 1]) - bg::get<1>(l.obj_[i]));
    length += sqrt(dx * dx + dy * dy);
    if (length >= s) {
      //! need to interpolate
      float segment_length = sqrt(dx * dx + dy * dy);
      float t = (length - s) / segment_length;
      float x_final =
        bg::get<0>(l.obj_[i]) * t + (1 - t) * bg::get<0>(l.obj_[i + 1]);
      float y_final =
        bg::get<1>(l.obj_[i]) * t + (1 - t) * bg::get<1>(l.obj_[i + 1]);
      return Point<T, 3>(x_final, y_final, atan2(dy, dx));
    }
  }
  return Point<T, 3>(0.0f, 0.0f, 0.0f);
}

}  // namespace geometry
