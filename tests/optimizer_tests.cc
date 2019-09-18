// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/optimizer.h"
#include "src/functors/follow_reference.h"


TEST(optimizer, basic_test) {
  using parameters::Parameters;
  using optimizer::Optimizer;
  using optimizer::FollowReference;
  using geometry::Matrix_t;

  Parameters params;
  params.set<double>("wheel_base", 1.7);

  Optimizer opt(&params);
  Matrix_t<double> opt_vec(3, 2);
  opt_vec << 0.0, 0.0,
             0.1, 0.0,
             0.2, 0.0;

  opt.SetOptimizationVector(opt_vec);
  FollowReference* reference_functor = new FollowReference();
  opt.AddResidualBlock(reference_functor);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
