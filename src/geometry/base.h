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

namespace geometry {

using std::string;
namespace bg = boost::geometry;
using boost::geometry::exterior_ring;
using boost::begin;
using boost::end;

//! points
template <typename T, int N>
using Point_t = bg::model::point<T, N, bg::cs::cartesian>;

//! linestring
template <typename T, int N>
using Linestring_t = bg::model::linestring<Point_t<T, N>>;

//! polygon
template <typename T, int N>
using Polygon_t = bg::model::polygon<Point_t<T, N>>;

//! State
template <typename T, int N>
using State_t = Eigen::Matrix<T, 1, N>;

//! Trajectory
template <typename T>
using Matrix_t = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<class Geometry>
struct BaseGeometry {
  BaseGeometry() :
    color_("blue"),
    style_("solid"),
    visibility_(true) {}
  explicit BaseGeometry(string color) :
    color_(color) {}
  BaseGeometry(string color, string style) :
    color_(color),
    style_(style),
    visibility_(true) {}
  BaseGeometry(string color, string style, bool visible) :
    color_(color),
    style_(style),
    visibility_(visible) {}
  virtual ~BaseGeometry() {}
  Geometry get() {
    return obj_;
  }
  void setColor(string color) { color_ = color; }
  void setStyle(string style) { style_ = style; }
  void setVisibility(bool visibility) { visibility_ = visibility; }
  void setBoundary(bool boundary) { boundary_ = boundary; }
  string getColor() const { return color_; }
  string getStyle() const { return style_; }
  bool getVisibility() const { return visibility_; }
  bool getBoundary() const { return boundary_; }
  string color_;
  string style_;
  bool visibility_;
  bool boundary_;
  Geometry obj_;
};

}  // namespace geometry
