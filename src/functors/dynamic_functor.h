
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Distance;
using geometry::Line;
using geometry::Point;
using geometry::Polygon;
using commons::Parameters;
using commons::CalculateJerk;
using dynamics::GenerateDynamicTrajectory;
using dynamics::SingleTrackModel;
using dynamics::NullModel;
using dynamics::IntegrationRK4;


template<class M, class I>
class DynamicFunctor : public BaseFunctor {
 public:
  explicit DynamicFunctor(Matrix_t<double> initial_state) :
    BaseFunctor(nullptr),
    initial_state_(initial_state) {}
  explicit DynamicFunctor(Matrix_t<double> initial_state,
                          Parameters* params) :
    BaseFunctor(params),
    initial_state_(initial_state) {}
  virtual ~DynamicFunctor() {}

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals, T cost = T(0.0)) {
    //! convert parameters to Eigen Matrix
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    
    Matrix_t<T> trajectory =
      GenerateDynamicTrajectory<T, M, I>(
        initial_state_t,
        opt_vec,
        params_);

    //! use boost ref line
    // Line<T, 2> ref_line(reference_line_.cast<T>());
    // T dist = CalculateDistance<T>(ref_line, trajectory);
    // cost += T(params_->get<double>("weight_distance", 0.1)) * dist * dist;


    //! calculate jerk
    /*
    T jerk = CalculateJerk<T>(
      trajectory,
      T(params_->get<double>("dt", 0.1)));
    cost += T(params_->get<double>("weight_jerk", 1000.0)) * jerk * jerk;
    residuals[0] = cost / (
      T(params_->get<double>("weight_distance", 0.1)) +
      T(params_->get<double>("weight_jerk", 1000.0)));*/
    
    residuals[0] = cost;
    return true;
  }

  Matrix_t<double> initial_state_;
};

typedef DynamicFunctor<SingleTrackModel, IntegrationRK4> SingleTrackFunctor;
typedef DynamicFunctor<NullModel, IntegrationRK4> NullModelFunctor;
}  // namespace optimizer
