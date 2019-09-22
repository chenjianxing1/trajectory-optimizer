
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <ceres/ceres.h>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"
#include "src/functors/base_functor.h"
#include "src/functors/costs/base_cost.h"
#include "src/functors/costs/jerk.h"
#include "src/functors/costs/reference.h"
#include "src/functors/costs/inputs.h"

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
using dynamics::IntegrationEuler;
using std::vector;


template<class M, class I>
class DynamicFunctor : public BaseFunctor {
 public:
  explicit DynamicFunctor(Matrix_t<double> initial_states) :
    BaseFunctor(nullptr),
    initial_states_(initial_states),
    costs_() {}
  explicit DynamicFunctor(Matrix_t<double> initial_states,
                          Parameters* params) :
    BaseFunctor(params),
    initial_states_(initial_states),
    costs_() {}
  virtual ~DynamicFunctor() {}

  template<typename T>
  bool operator()(T const* const* parameters,
                  T* residuals,
                  T costs = T(0.),
                  T weights = T(0.)) {
    // conversion
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_states_t = initial_states_.cast<T>();

    // generation
    Matrix_t<T> trajectory = GenerateDynamicTrajectory<T, M, I>(
      initial_states_t,
      opt_vec,
      params_);

    // costs
    for ( int i = 0; i < costs_.size(); i++ ) {
      if (dynamic_cast<JerkCost*>(costs_[i])) {
        JerkCost* cost = dynamic_cast<JerkCost*>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
      }
      if (dynamic_cast<ReferenceCost*>(costs_[i])) {
        ReferenceCost* cost = dynamic_cast<ReferenceCost*>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
      }
      if (dynamic_cast<InputCost*>(costs_[i])) {
        InputCost* cost = dynamic_cast<InputCost*>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
      }
      // TODO(@hart): boundaries, dynamic objects
    }

    residuals[0] = costs / weights;
    return true;
  }

  void AddCost(BaseCost* cost) {
    costs_.push_back(cost);
  }

  Matrix_t<double> initial_states_;
  vector<BaseCost*> costs_;
};

typedef DynamicFunctor<SingleTrackModel, IntegrationRK4> SingleTrackFunctor;
typedef DynamicFunctor<SingleTrackModel,
                       IntegrationEuler> FastSingleTrackFunctor;
typedef DynamicFunctor<NullModel, IntegrationRK4> NullModelFunctor;
}  // namespace optimizer
