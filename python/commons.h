// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once

#include <string>
#include <vector>
#include "src/commons/commons.h"
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"

namespace py = pybind11;

void python_commons(py::module m) {
  using commons::Parameter;
  using commons::ParameterPtr;
  using commons::CalculateSquaredJerk;
  using commons::ObjectOutline;

  py::class_<Parameter, ParameterPtr>(m, "Parameter")
    .def(py::init<>())
    .def("get", &commons::Parameter::get<double>)
    .def("get", &commons::Parameter::get<int>)
    .def("get", &commons::Parameter::get<std::string>)
    .def("get", &commons::Parameter::get<bool>)
    .def("get", &commons::Parameter::get<std::vector<double>>)
    .def("set", &commons::Parameter::set<double>)
    .def("set", &commons::Parameter::set<int>)
    .def("set", &commons::Parameter::set<std::string>)
    .def("set", &commons::Parameter::set<bool>)
    .def("set", &commons::Parameter::set<std::vector<double>>);
  
  m.def("CalculateSquaredJerk", &commons::CalculateSquaredJerk<double>);

  py::class_<ObjectOutline>(m, "ObjectOutline")
    .def(py::init<>())
    .def(py::init<const geometry::Matrix_t<double>&, double>())
    .def("Add", &commons::ObjectOutline::Add)
    .def("Query", &commons::ObjectOutline::Query);
}