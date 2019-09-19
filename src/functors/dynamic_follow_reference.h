
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Distance;
using geometry::Collides;
using geometry::Line;
using geometry::Polygon;
using commons::Parameters;
using optimizer::BaseFunctor;
using dynamics::GenerateDynamicTrajectory;
using dynamics::SingleTrackModel;
using dynamics::integrationRK4;
using dynamics::integrationEuler;


class DynamicModelFollowReference : public BaseFunctor {
 public:
  DynamicModelFollowReference() : BaseFunctor(nullptr) {}
  explicit DynamicModelFollowReference(Matrix_t<double> initial_state,
                           Parameters* params) :
    initial_state_(initial_state),
    BaseFunctor(params) {}
  virtual ~DynamicModelFollowReference() {}

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals) {
    T cost = T(0.0);
    //! convert parameters to Eigen Matrix
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    // either (x, y) or (x, y, theta, v)
    Matrix_t<T> trajectory =
      GenerateDynamicTrajectory<T, SingleTrackModel<T, integrationRK4>>(
        initial_state_t, opt_vec, *this->GetParams());

    //! use boost ref line
    // Point2d_t<T>
    // Line_t<T> bark_line;
    /*
    Line<T, 2> boost_line;  // TODO(@hart); make eigen passable
    Point<T, 2> pt(T(0.), T(1.0));
    Point<T, 2> pt1(T(10.), T(5.0));
    Point<T, 2> pt2(T(15.), T(3.0));
    boost_line.Append(pt);
    boost_line.Append(pt1);*/
    Polygon<T, 2> boost_poly;
    Point<T, 2> pt(T(0.), T(0.0));
    Point<T, 2> pt1(T(5.), T(0.0));
    Point<T, 2> pt2(T(5.), T(5.0));
    Point<T, 2> pt3(T(0.), T(5.));
    Point<T, 2> pt4(T(0.), T(.0));
    Point<T, 2> pt5(T(110.), T(110.));

    boost_poly.Append(pt);
    boost_poly.Append(pt1);
    boost_poly.Append(pt2);
    boost_poly.Append(pt3);
    boost_poly.Append(pt4);

    //std::cout << Collides<Polygon<T, 2>, Point<T, 2>>(boost_poly, pt5);
    std::cout << Distance<T, Polygon<T, 2>, Point<T, 2>>(boost_poly, pt5);

    //! calculate jerk
    T jerk = CalculateJerk<T>(
      trajectory,
      T(this->GetParams()->get<double>("dt", 0.1)));

    cost += T(this->GetParams()->get<double>("weight_jerk", 1000.0)) * jerk * jerk;
    residuals[0] = cost / (
      T(this->GetParams()->get<double>("weight_distance", 0.1)) +
      T(this->GetParams()->get<double>("weight_jerk", 1000.0)));
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

 private:
  Matrix_t<double> reference_line_;
  Matrix_t<double> initial_state_;
  
};

}  // namespace optimizer
