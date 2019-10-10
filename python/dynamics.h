// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include "src/dynamics/dynamics.h"

namespace py = pybind11;

void python_dynamics(py::module m) {
  using dynamics::GenerateDynamicTrajectory;
  using dynamics::SingleTrackModel;
  using dynamics::TripleIntModel;
  using dynamics::RobotArm;
  using dynamics::IntegrationRK4;

  m.def("GenerateTrajectorySingleTrack",
    &dynamics::GenerateDynamicTrajectory<double,
                                        SingleTrackModel,
                                        IntegrationRK4>);
  m.def("GenerateTrajectoryTripleInt",
    &dynamics::GenerateDynamicTrajectory<double,
                                        TripleIntModel,
                                        IntegrationRK4>);
  m.def("GenerateTrajectoryRobotArm",
    &dynamics::GenerateDynamicTrajectory<double,
                                        RobotArm,
                                        IntegrationRK4>);
}