
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"

namespace optimizer {

using geometry::Matrix_t;
using parameters::Parameters;

class FollowReference {
 public:
  explicit FollowReference(Parameters* params) :
    params_(params) {}

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
  Parameters* params_;
  Matrix_t<double> reference_line_;
};

}  // namespace optimizer
