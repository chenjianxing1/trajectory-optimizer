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
  using dynamics::IntegrationRK4;
  using dynamics::IntegrationEuler;
  using geometry::Matrix_t;
  using commons::Parameters;

  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> state(1, 4);
  state << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  Matrix_t<double> inp(1, 2);
  inp << 0.0, 0.0;  // acceleration and steering angle

  SingleTrackModel model(&params);
  state = model.Step<double, IntegrationRK4>(state, inp, &params);
  Matrix_t<double> state_after(1, 4);
  state_after << 0.5, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(state, state_after);
  state = model.Step<double, IntegrationEuler>(state, inp, &params);
  Matrix_t<double> state_after_again(1, 4);
  state_after_again << 1.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(state, state_after_again);

}

TEST(dynamics, traj_gen) {
  using dynamics::SingleTrackModel;
  using dynamics::IntegrationRK4;
  using dynamics::GenerateDynamicTrajectory;
  using geometry::Matrix_t;
  using commons::Parameters;

  Parameters params;
  params.set<double>("wheel_base", 2.7);
  params.set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> state(1, 4);
  state << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  Matrix_t<double> inp(3, 2);
  inp << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0;  // acceleration and steering angle

  SingleTrackModel model(&params);
  Matrix_t<double> trajectory =
    GenerateDynamicTrajectory<double, SingleTrackModel, IntegrationRK4>(
      state,
      inp,
      &params);
  std::cout << trajectory << std::endl;

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}