
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
#include "src/functors/costs/distance.h"
#include "src/functors/costs/reference.h"
#include "src/functors/costs/static_object.h"
#include "src/functors/costs/speed.h"
#include "src/functors/costs/inputs.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Distance;
using geometry::Line;
using geometry::Point;
using geometry::Polygon;
using commons::Parameter;
using commons::ParameterPtr;
using commons::CalculateSquaredJerk;
using dynamics::GenerateDynamicTrajectory;
using dynamics::SingleTrackModel;
using dynamics::NullModel;
using dynamics::IntegrationRK4;
using dynamics::IntegrationEuler;
using std::vector;


/**
 * @brief A functor for dynamic optimization
 * 
 * @tparam M Used model (e.g. SingleTrackModel)
 * @tparam I Used integration method (e.g. Explicit Euler)
 */
template<class M, class I>
class DynamicFunctor : public BaseFunctor {
 public:
  explicit DynamicFunctor(Matrix_t<double> initial_states) :
    BaseFunctor(nullptr),
    initial_states_(initial_states) {}

  explicit DynamicFunctor(Matrix_t<double> initial_states,
                          ParameterPtr params) :
    BaseFunctor(params),
    initial_states_(initial_states) {}

  /**
   * @brief Function that is called by the ceres-solver
   * 
   * @tparam T Type of data
   * @param parameters Parameter class
   * @param residuals Residuals that will be optimized
   * @param costs Calculated costs
   * @param weights Calculated weights
   * @return true Whether the optimization was successful
   */
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
      params_.get());
    // TODO(@hart): we need the state information here as well;
    //              otherwise cannot be used in cost terms
    // M m;
    // std::cout << m.state_def_->x() << std::endl;

    // costs
    vector<BaseCostPtr> costs_ = this->GetSquaredObjectCosts();
    for ( int i = 0; i < costs_.size(); i++ ) {
      if (std::dynamic_pointer_cast<JerkCost>(costs_[i])) {
        // would need state def
        JerkCostPtr cost = std::dynamic_pointer_cast<JerkCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
      if (std::dynamic_pointer_cast<ReferenceLineCost>(costs_[i])) {
        // would need state def
        ReferenceLineCostPtr cost =
          std::dynamic_pointer_cast<ReferenceLineCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
      if (std::dynamic_pointer_cast<InputCost>(costs_[i])) {
        InputCostPtr cost = std::dynamic_pointer_cast<InputCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
      if (std::dynamic_pointer_cast<ReferenceCost>(costs_[i])) {
        ReferenceCostPtr cost =
          std::dynamic_pointer_cast<ReferenceCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
      if (std::dynamic_pointer_cast<SpeedCost>(costs_[i])) {
        // would need state def
        SpeedCostPtr cost =
          std::dynamic_pointer_cast<SpeedCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
      if (std::dynamic_pointer_cast<StaticObjectCost>(costs_[i])) {
        StaticObjectCostPtr cost =
          std::dynamic_pointer_cast<StaticObjectCost>(costs_[i]);
        costs += cost->Evaluate<T>(trajectory, opt_vec);
        weights += cost->Weight<T>();
        continue;
      }
    }

    residuals[0] = costs / weights;
    return true;
  }

 private:
  Matrix_t<double> initial_states_;
};

typedef DynamicFunctor<SingleTrackModel, IntegrationRK4> SingleTrackFunctor;
typedef DynamicFunctor<SingleTrackModel,
                       IntegrationEuler> FastSingleTrackFunctor;
typedef DynamicFunctor<NullModel, IntegrationRK4> NullModelFunctor;

typedef std::shared_ptr<SingleTrackFunctor> SingleTrackFunctorPtr;
typedef std::shared_ptr<FastSingleTrackFunctor> FastSingleTrackFunctorPtr;
typedef std::shared_ptr<NullModelFunctor> NullModelFunctorPtr;
}  // namespace optimizer
