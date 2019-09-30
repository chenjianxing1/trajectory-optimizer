// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <Eigen/Dense>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/dynamics/dynamics.h"
#include "src/dynamics/integration/rk4.h"
#include "src/dynamics/integration/euler.h"

namespace dynamics {

using geometry::Matrix_t;
using commons::ParameterPtr;
using commons::Parameter;


/**
 * @brief Tripple integrator model
 * 
 */
class TripleIntModel {
 public:
  TripleIntModel() {}

  enum class StateDefinition {
    X = 0,
    VX = 1,
    AX = 2,
    Y = 3,
    VY = 4,
    AY = 5,
    Z = 6,
    VZ = 7,
    AZ = 8
  };

  enum class InputDefinition {
    AX = 0,
    AY = 1,
    AZ = 2
  };

  template<typename T>
  static Matrix_t<T> fDot(const Matrix_t<T>& state,
                          const Matrix_t<T>& u) {
    Matrix_t<T> A(9, 9);
    Matrix_t<T> B(9, 3);
    A << T(0.), T(1.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(1.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(1.), T(0.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(1.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(1.), T(0.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(1.),
         T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.), T(0.);
    B << T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.),
         T(1.), T(0.), T(0.),
         T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.),
         T(0.), T(1.), T(0.),
         T(0.), T(0.), T(0.),
         T(0.), T(0.), T(0.),
         T(0.), T(0.), T(1.);
    return (A*state.transpose() + B*u.transpose()).transpose();
  }

  template<typename T, class I>
  static Matrix_t<T> Step(const Matrix_t<T>& state,
                          const Matrix_t<T>& u,
                          Parameter* params) {
    std::function<Matrix_t<T> (const Matrix_t<T>&)> fDot_ =
      std::bind(fDot<T>,
                std::placeholders::_1,
                u);
    return I::template Integrate<T>(state,
                                    fDot_,
                                    T(params->get<double>("dt", 0.2)));
  }

};

}  // namespace dynamics
