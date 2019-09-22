// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/complex.h"
#include "pybind11/stl_bind.h"
#include "pybind11/eigen.h"
#include "boost/variant.hpp"

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
namespace py = pybind11;

PYBIND11_MODULE(bark, m) {
  m.doc() = "Python wrappings.";
  python_runtime(m.def_submodule("runtime", "submodule containing the runtime"));
}