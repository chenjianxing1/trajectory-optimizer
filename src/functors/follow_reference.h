
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using parameters::Parameters;
using optimizer::BaseFunctor;
using dynamics::InputToTrajectory;
using dynamics::SingleTrackModel;
using dynamics::integrationRK4;
using dynamics::integrationEuler;

class FollowReference : public BaseFunctor {
 public:
  FollowReference() : BaseFunctor(nullptr) {}
  explicit FollowReference(Matrix_t<double> initial_state,
                           Parameters* params) :
    initial_state_(initial_state),
    BaseFunctor(params) {}
  virtual ~FollowReference() {}

  template<typename T>
  bool operator()(T const* const* parameters, T* residuals) {
    T cost = T(0.0);
    //! convert parameters to Eigen Matrix
    Matrix_t<T> opt_vec = this->ParamsToEigen<T>(parameters);
    Matrix_t<T> initial_state_t = initial_state_.cast<T>();

    // either (x, y) or (x, y, theta, v)
    // TODO(@hart): needs to be passed
    Matrix_t<T> trajectory =
      InputToTrajectory<T, SingleTrackModel<T, integrationRK4>>(
        initial_state_t, opt_vec, *this->GetParams());

    // TODO(@hart): calculate the costs based on the trajectory


    residuals[0] = cost;
    return true;
  }

  void SetReferenceLine(const Matrix_t<double>& line) {
    reference_line_ = line;
  }

 private:
  Matrix_t<double> reference_line_;
  Matrix_t<double> initial_state_;
};

}  // namespace optimizer
