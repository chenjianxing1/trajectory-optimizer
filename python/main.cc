// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/complex.h"
#include "pybind11/stl_bind.h"
#include "pybind11/eigen.h"
#include "boost/variant.hpp"

// optimizer specific includes
#include "python/commons.hpp"
#include "python/geometry.hpp"
#include "python/dynamics.hpp"
#include "python/optimizer.hpp"

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
namespace py = pybind11;

PYBIND11_MODULE(optimizer, m) {
  m.doc() = "Python wrappings.";
  python_optimizer(m.def_submodule("optimizer", ".."));
  python_geometry(m.def_submodule("geometry", ".."));
  python_dynamics(m.def_submodule("dynamics", ".."));
  python_commons(m.def_submodule("commons", ".."));
}