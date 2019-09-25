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

using namespace optimizer;
using namespace commons;
using namespace dynamics;

void python_optimizer(py::module m) {
  py::class_<BaseFunctor, BaseFunctorPtr>(m, "BaseFunctor")
    .def(py::init<const ParameterPtr&>());

  py::class_<SingleTrackFunctor,
             BaseFunctor,
             SingleTrackFunctorPtr>(m, "SingleTrackFunctor")
    .def(py::init<Matrix_t<double>, const ParameterPtr&>())
    .def("AddCost", &SingleTrackFunctor::AddCost);

  py::class_<BaseCost, BaseCostPtr>(m, "BaseCost")
    .def(py::init<const ParameterPtr& >());

  py::class_<JerkCost, BaseCost, JerkCostPtr>(m, "JerkCost")
    .def(py::init<const ParameterPtr&>());

  py::class_<ReferenceCost, BaseCost, ReferenceCostPtr>(m, "ReferenceCost")
    .def(py::init<const ParameterPtr&>())
    .def("SetReferenceLine", &optimizer::ReferenceCost::SetReferenceLine);

  py::class_<InputCost, BaseCost, InputCostPtr>(m, "InputCost")
    .def(py::init<const ParameterPtr&>());

  py::class_<Optimizer, std::shared_ptr<Optimizer>>(m, "Optimizer")
    .def(py::init<const ParameterPtr&>())
    // .def("AddResidualBlock",
    //   &optimizer::Optimizer::AddResidualBlock<SingleTrackFunctor, 4>)
    .def("Solve", &optimizer::Optimizer::Solve)
    .def("GetOptimizationVector", &optimizer::Optimizer::GetOptimizationVector)
    .def("SetOptimizationVector", &optimizer::Optimizer::SetOptimizationVector)
    .def("AddSingleTrackFunctor",
      &optimizer::Optimizer::PythonAddSingleTrackFunctor<SingleTrackFunctor>)
    .def("Report", &optimizer::Optimizer::Report);
}