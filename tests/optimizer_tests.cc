// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"
#include "src/optimizer.h"
#include "src/functors/dynamic_functor.h"
#include "src/functors/base_functor.h"
#include "src/functors/costs/jerk.h"
#include "src/functors/costs/base_cost.h"
#include "src/functors/costs/reference.h"
#include "src/functors/costs/inputs.h"


TEST(optimizer, single_track_model) {
  using commons::Parameter;
  using commons::ParameterPtr;
  using optimizer::Optimizer;
  using optimizer::JerkCost;
  using optimizer::BaseCost;
  using optimizer::BaseCostPtr;
  using optimizer::BaseFunctor;
  using optimizer::BaseFunctorPtr;
  using optimizer::JerkCostPtr;
  using optimizer::InputCostPtr;
  using optimizer::InputCost;
  using optimizer::ReferenceCost;
  using optimizer::ReferenceCostPtr;
  using optimizer::SingleTrackFunctor;
  using optimizer::FastSingleTrackFunctor;
  using optimizer::FastSingleTrackFunctorPtr;
  using optimizer::NullModelFunctor;
  using geometry::Matrix_t;
  using dynamics::SingleTrackModel;
  using dynamics::IntegrationRK4;
  using dynamics::IntegrationEuler;
  using dynamics::GenerateDynamicTrajectory;

  // initialization
  ParameterPtr params = std::make_shared<Parameter>();
  params->set<double>("wheel_base", 2.7);
  params->set<double>("dt", 0.2);

  // weights
  params->set<double>("weight_jerk", 20.);
  params->set<double>("weight_distance", 100.);
  params->set<double>("function_tolerance", 1e-6);

  Matrix_t<double> initial_states(3, 4);
  initial_states << 0.0, 0.0, 0.0, 10.0,
                    1.0, 0.0, 0.0, 10.0,
                    2.0, 0.0, 0.0, 10.0;  // x, y, theta, v

  Matrix_t<double> opt_vec(20, 2);
  opt_vec.setZero();

  Matrix_t<double> ref_line(2, 2);
  ref_line << 0., 0.,
              1000., 0.;


  // optimization
  Optimizer opt(params);
  opt.SetOptimizationVector(opt_vec);

  // add reference functor
  BaseFunctor* functor =
    new SingleTrackFunctor(initial_states, params);

  // jerk
  JerkCostPtr jerk_costs = std::make_shared<JerkCost>(params);
  functor->AddCost(jerk_costs);

  // reference line
  ReferenceCostPtr ref_costs = std::make_shared<ReferenceCost>(params);
  ref_costs->SetReferenceLine(ref_line);
  functor->AddCost(ref_costs);

  // input constraints
  InputCostPtr inp_costs = std::make_shared<InputCost>(params);
  Matrix_t<double> lb(1, 2);
  lb << -0.2, -1.0;
  Matrix_t<double> ub(1, 2);
  ub << 0.2, 1.0;
  inp_costs->SetLowerBound(lb);
  inp_costs->SetUpperBound(ub);
  functor->AddCost(inp_costs);

  opt.AddResidualBlock<SingleTrackFunctor>(functor);


  // solve
  opt.Solve();
  opt.Report();
  std::cout << opt.Result() << std::endl;

  // trajectory
  Matrix_t<double> trajectory =
    GenerateDynamicTrajectory<double, SingleTrackModel, IntegrationRK4>(
      initial_states,
      opt.Result(),
      params.get());
  std::cout << trajectory << std::endl;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
