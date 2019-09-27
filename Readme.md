# Dynamic Trajectory Optimization

This is a library for dynamic trajectory optimization for autonomous agents. This library is capable of producing dynamically feasible trajectories using several dynamic models and integration methods.
We use the [ceres-solver]() to solve the non-linear optimization problem in an unconstrained optimization setting.

Implemented models:
* Simplified single track model
* Null model

Implemented integration methods:

* Explicit Euler method
* Runge-Kutta method (4th order)

## Getting Started

To get started you will need to have [bazel]() and [Python 3.6]() installed. Next, set up the virtual python environment in order to build and use the Python bindings for the optimization library.
This can be done using `bash install.sh` and then entering the environment using the command `bash dev_into.sh`. Once you are in the environment make sure all tests run without any error by running the bazel command
`bazel test //...`. In order to see an exemplary output of the optimization library you then can run the command `bazel run //tests:py_optimizer_single_track_tests`.

(C) Copyright by Patrick Hart (patrick.hart@tum.de)