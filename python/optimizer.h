// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <memory>
#include <ceres/ceres.h>
#include "src/commons/commons.h"
#include "src/functors/base_functor.h"
#include "src/functors/dynamic_functor.h"
#include "src/functors/costs/jerk.h"
#include "src/functors/costs/inputs.h"
#include "src/functors/costs/reference.h"
#include "src/optimizer.h"

namespace py = pybind11;
using optimizer::Optimizer;
using optimizer::BaseFunctor;
using optimizer::SingleTrackFunctor;
using optimizer::FastSingleTrackFunctor;
using optimizer::InputCost;
using optimizer::JerkCost;
using optimizer::BaseCost;
using optimizer::ReferenceCost;
using commons::Parameters;
using geometry::Matrix_t;


void python_optimizer(py::module m) {
  py::class_<BaseFunctor>(m, "BaseFunctor")
    .def(py::init<Parameters*>());

  py::class_<SingleTrackFunctor,
             BaseFunctor>(m, "SingleTrackFunctor")
    .def(py::init<Matrix_t<double>, Parameters*>())
    .def("AddCost", &SingleTrackFunctor::AddCost);

  py::class_<BaseCost>(m, "BaseCost")
    .def(py::init<Parameters*>());

  py::class_<JerkCost, BaseCost>(m, "JerkCost")
    .def(py::init<Parameters*>());

  py::class_<ReferenceCost, BaseCost>(m, "ReferenceCost")
    .def(py::init<Parameters*>());

  py::class_<InputCost, BaseCost>(m, "InputCost")
    .def(py::init<Parameters*>());

  py::class_<Optimizer>(m, "Optimizer")
    .def(py::init<Parameters*>())
    .def("AddResidualBlock",
      &optimizer::Optimizer::AddResidualBlock<SingleTrackFunctor,
                                              ceres::TrivialLoss,
                                              4>)
    .def("Solve", &optimizer::Optimizer::Solve)
    .def("GetOptimizationVector", &optimizer::Optimizer::GetOptimizationVector)
    .def("SetOptimizationVector", &optimizer::Optimizer::SetOptimizationVector)
    .def("Report", &optimizer::Optimizer::Report);
}