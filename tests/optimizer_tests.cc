// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"
#include "src/optimizer.h"
#include "src/functors/base_functor.h"
#include "src/functors/follow_reference.h"
#include "src/functors/dynamic_follow_reference.h"


TEST(optimizer, basic_test) {
  using commons::Parameters;
  using optimizer::Optimizer;
  using optimizer::BaseFunctor;
  using optimizer::FollowReference;
  using optimizer::DynamicModelFollowReference;
  using geometry::Matrix_t;
  using dynamics::SingleTrackModel;

  // initialization
  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.2);


  Matrix_t<double> initial_state(1, 4);
  initial_state << 0.0, 0.0 , 0.0, 5.0;  // x, y, theta, v
  Matrix_t<double> opt_vec(6, 2);
  opt_vec << 0.0, 0.0,
             0.1, 0.0,
             0.2, 0.0,
             0.2, 0.0,
             0.2, 0.0,
             0.2, 0.0;

  // optimization
  Optimizer opt(&params);
  opt.SetOptimizationVector(opt_vec);

  // add functor
  DynamicModelFollowReference* reference_functor =
    new DynamicModelFollowReference(initial_state,
                                    &params);
  opt.AddResidualBlock<DynamicModelFollowReference>(reference_functor);

  // solving
  opt.Solve();
  opt.Report();
  std::cout << opt.GetOptimizationVector() << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
