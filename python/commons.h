// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once

#include <string>
#include <vector>
#include "src/commons/commons.h"
#include "src/commons/parameters.h"

namespace py = pybind11;

void python_commons(py::module m) {
  using commons::Parameters;

  py::class_<Parameters>(m, "Parameters")
    .def(py::init<>())
    .def("get", &commons::Parameters::get<double>)
    .def("get", &commons::Parameters::get<int>)
    .def("get", &commons::Parameters::get<std::string>)
    .def("get", &commons::Parameters::get<bool>)
    .def("get", &commons::Parameters::get<std::vector<double>>)
    .def("set", &commons::Parameters::set<double>)
    .def("set", &commons::Parameters::set<int>)
    .def("set", &commons::Parameters::set<std::string>)
    .def("set", &commons::Parameters::set<bool>)
    .def("set", &commons::Parameters::set<std::vector<double>>);
}