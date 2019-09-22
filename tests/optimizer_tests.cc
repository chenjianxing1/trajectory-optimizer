// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"
#include "src/optimizer.h"
#include "src/functors/base_functor.h"
#include "src/functors/dynamic_functor.h"
#include "src/functors/costs/jerk.h"
#include "src/functors/costs/reference.h"


TEST(optimizer, basic_test) {
  using commons::Parameters;
  using optimizer::Optimizer;
  using optimizer::BaseFunctor;
  using optimizer::JerkCost;
  using optimizer::DynamicFunctor;
  using optimizer::ReferenceCost;
  using optimizer::SingleTrackFunctor;
  using optimizer::FastSingleTrackFunctor;
  using geometry::Matrix_t;
  using dynamics::SingleTrackModel;
  using dynamics::IntegrationRK4;
  using dynamics::IntegrationEuler;
  using dynamics::GenerateDynamicTrajectory;

  // initialization
  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.1);

  // weights
  params.set<double>("weight_jerk", 1e3);
  params.set<double>("weight_distance", 1);
  params.set<double>("function_tolerance", 1e-9);

  Matrix_t<double> initial_states(1, 4);
  initial_states << 0.0, 0.0 , 0.0, 5.0;  // x, y, theta, v

  Matrix_t<double> opt_vec(6, 2);
  opt_vec.setZero();

  Matrix_t<double> ref_line(3, 2);
  ref_line << 0., 0.,
              5., 0.5,
              10., 0.;


  // optimization
  Optimizer opt(&params);
  opt.SetOptimizationVector(opt_vec);

  // add reference functor
  FastSingleTrackFunctor* functor =
    new FastSingleTrackFunctor(initial_states, &params);

  // jerk
  JerkCost* jerk_costs = new JerkCost(&params);
  functor->AddCost(jerk_costs);

  // reference line
  ReferenceCost* ref_costs = new ReferenceCost(&params);
  ref_costs->SetReferenceLine(ref_line);
  functor->AddCost(ref_costs);


  opt.AddResidualBlock<FastSingleTrackFunctor>(functor);

  // fix first two opt vec params
  // opt.FixOptimizationVector(0, 2);

  // solve
  opt.Solve();
  opt.Report();
  std::cout << opt.GetOptimizationVector() << std::endl;

  // trajectory
  Matrix_t<double> trajectory =
    GenerateDynamicTrajectory<double,  SingleTrackModel, IntegrationRK4>(
      initial_states,
      opt.GetOptimizationVector(),
      &params);
  std::cout << trajectory << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
