// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <functional>
#include <memory>
#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"


TEST(dynamics, single_track_model) {
  using dynamics::SingleTrackModel;
  using dynamics::IntegrationRK4;
  using dynamics::IntegrationEuler;
  using geometry::Matrix_t;
  using commons::Parameter;
  using commons::ParameterPtr;

  ParameterPtr params = std::make_shared<Parameter>();
  params->set<double>("wheel_base", 2.7);
  params->set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> state(1, 4);
  state << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  Matrix_t<double> inp(1, 2);
  inp << 0.0, 0.0;  // acceleration and steering angle

  SingleTrackModel model;
  state = model.Step<double, IntegrationRK4>(state, inp, params.get());
  Matrix_t<double> state_after(1, 4);
  state_after << 0.5, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(state, state_after);
  state = model.Step<double, IntegrationEuler>(state, inp, params.get());
  Matrix_t<double> state_after_again(1, 4);
  state_after_again << 1.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  ASSERT_EQ(state, state_after_again);
}

TEST(dynamics, triple_int_model) {
  using dynamics::TripleIntModel;
  using dynamics::IntegrationRK4;
  using geometry::Matrix_t;
  using commons::Parameter;
  using commons::ParameterPtr;

  ParameterPtr params = std::make_shared<Parameter>();
  params->set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> state(1, 9);
  // x, vx, vy, y, vy, ay, z, vz, az
  state << 0.0, 1.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 1.0, 0.0;
  Matrix_t<double> inp(1, 3);
  inp << 1.0, 1.0, 1.0;  // ax, ay, az

  TripleIntModel model;
  state = model.Step<double, IntegrationRK4>(state, inp, params.get());
  std::cout << state << std::endl;
}

TEST(dynamics, robot_arm) {
  using dynamics::RobotArm;
  using dynamics::IntegrationRK4;
  using geometry::Matrix_t;
  using commons::Parameter;
  using commons::ParameterPtr;
  using dynamics::GenerateDynamicTrajectory;

  ParameterPtr params = std::make_shared<Parameter>();
  params->set<double>("dt", 0.1);
  params->set<bool>("static", true);
  //! add objects to world
  Matrix_t<double> state(1, 2);
  state << 0.0, 0.0;

  Matrix_t<double> inp(1, 2);
  inp << .0, .0;  // theta0, theta1

  RobotArm model;
  state = model.Step<double, IntegrationRK4>(state, inp, params.get());
  std::cout << state << std::endl;

  inp << 1.54, 1.54;  // theta0, theta1
  state = model.Step<double, IntegrationRK4>(state, inp, params.get());
  std::cout << state << std::endl;


  inp.resize(3, 2);
  inp << 0.0, 0.0,
         1.54, 1.54,
         0.0, 0.0;  // theta0, theta1
  state << 1.0, 1.0;

  Matrix_t<double> trajectory =
    GenerateDynamicTrajectory<double, RobotArm, IntegrationRK4>(
      state,
      inp,
      params.get());
  std::cout << "Trajectory: \n" << trajectory << std::endl;

}

TEST(dynamics, traj_gen) {
  using dynamics::SingleTrackModel;
  using dynamics::IntegrationRK4;
  using dynamics::IntegrationEuler;
  using dynamics::GenerateDynamicTrajectory;
  using geometry::Matrix_t;
  using commons::Parameter;
  using commons::ParameterPtr;

  ParameterPtr params = std::make_shared<Parameter>();
  params->set<double>("wheel_base", 2.7);
  params->set<double>("dt", 0.1);

  //! add objects to world
  Matrix_t<double> initial_states(3, 4);
  initial_states << 0.0, 0.0, 0.0, 10.0,
                    1.0, 0.0, 0.0, 10.0,
                    2.0, 0.0, 0.0, 10.0;  // x, y, theta, v
  Matrix_t<double> inp(3, 2);
  inp << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0;  // acceleration and steering angle

  Matrix_t<double> trajectory =
    GenerateDynamicTrajectory<double, SingleTrackModel, IntegrationRK4>(
      initial_states,
      inp,
      params.get());
  std::cout << trajectory << std::endl;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}