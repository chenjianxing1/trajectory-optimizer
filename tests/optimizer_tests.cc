// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <ceres/ceres.h>
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/optimizer.h"
#include "src/functors/follow_reference.h"


TEST(optimizer, basic_test) {
  using parameters::Parameters;
  using optimizer::Optimizer;
  using optimizer::FollowReference;
  using geometry::Matrix_t;
  using ceres::DynamicAutoDiffCostFunction;

  Parameters params;
  params.set<double>("wheel_base", 1.7);

  Optimizer opt(&params);
  Matrix_t<double> opt_vec(3, 2);
  opt_vec << 0.0, 0.0,
             0.1, 0.0,
             0.2, 0.0;

  opt.SetOptimizationVector(opt_vec);

  FollowReference* reference_functor = new FollowReference(&params);

  // TODO(@hart): this code I would rather not want to see
  // handle internally
  DynamicAutoDiffCostFunction<FollowReference, 4>* cost_function =
      new DynamicAutoDiffCostFunction<FollowReference, 4>(reference_functor);

  opt.AddResidualBlock<FollowReference>(cost_function);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
