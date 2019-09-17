// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <functional>
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"


TEST(dynamics, single_track_model) {
  using dynamics::SingleTrackModel;
  using dynamics::integrationRK4;
  using geometry::Matrix_t;
  using parameters::Parameters;

  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> dynamic_state(1, 4);
  dynamic_state << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v

  Matrix_t<double> inp(1, 2);
  inp << 0.0, 0.0;  // acceleration and steering angle

  dynamic_state = SingleTrackModel<double,
                                   integrationRK4>(dynamic_state,
                                                   inp,
                                                   params);
  Matrix_t<double> dynamic_state_after(1, 4);
  dynamic_state_after << 0.5, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(dynamic_state, dynamic_state_after);

  dynamic_state = SingleTrackModel<double,
                                  integrationRK4>(dynamic_state,
                                                  inp,
                                                  params);
  Matrix_t<double> dynamic_state_after_again(1, 4);
  dynamic_state_after_again << 1.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(dynamic_state, dynamic_state_after_again);
}

TEST(dynamics, trajectory_generation) {
  using dynamics::SingleTrackModel;
  using dynamics::integrationRK4;
  using dynamics::GenerateTrajectory;
  using geometry::Matrix_t;
  using parameters::Parameters;

  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> dynamic_state(1, 4);
  dynamic_state << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v

  Matrix_t<double> inp(2, 2);
  inp << 0.0, 0.0,
         0.1, 0.0;  // acceleration and steering angle x2


  Matrix_t<double> trajectory =
    GenerateTrajectory<double, SingleTrackModel<double, integrationRK4>>(
      dynamic_state,
      inp,
      params);
  Matrix_t<double> trajectory_after(2, 4);
  trajectory_after << 0.0, 0.0, 0.0, 5.0,
                      0.5, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(trajectory, trajectory_after);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}