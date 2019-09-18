
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"
#include "src/commons/parameters.h"

namespace optimizer {

using geometry::Matrix_t;
using parameters::Parameters;
using optimizer::BaseFunctor;

class FollowReference : public BaseFunctor {
 public:
  FollowReference() : BaseFunctor(nullptr) {}
  explicit FollowReference(Parameters* params) :
    BaseFunctor(params) {}
  virtual ~FollowReference() {}

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals) {
    T cost = T(0.0);

    // returns Eigen matrix
    // either (x, y) or (x, y, theta, v)
    // Matrix_t<T> traj = kinematics(parameters, _trajectory);

    residuals[0] = cost;
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

 private:
  Matrix_t<double> reference_line_;
};

}  // namespace optimizer
