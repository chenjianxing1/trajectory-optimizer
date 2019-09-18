
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <ceres/ceres.h>
#include "src/commons/parameters.h"
#include "src/geometry/geometry.h"
#include "src/functors/base_functor.h"

namespace optimizer {

using parameters::Parameters;
using geometry::Matrix_t;
using optimizer::BaseFunctor;
using std::vector;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::TrivialLoss;

class Optimizer {
 public:
  explicit Optimizer(Parameters* params) :
    optimization_vectors_(),
    parameter_block_(),
    params_(params) {}

  template<typename T, int N = 4>
  void AddResidualBlock(
    BaseFunctor* functor,
    LossFunction* loss = new TrivialLoss(),
    int num_residuals = 1) {
    DynamicAutoDiffCostFunction<T, N>* ceres_functor =
      new DynamicAutoDiffCostFunction<T, N>(dynamic_cast<T*>(functor));
    for (vector<double>& vec : optimization_vectors_) {
      ceres_functor->AddParameterBlock(vec.size());
    }
    ceres_functor->SetNumResiduals(num_residuals);
    problem_.AddResidualBlock(ceres_functor, loss, parameter_block_);
  }

  // each column of an Eigen Matrix will become an optimization vector
  void SetOptimizationVector(Matrix_t<double> inputs) {
    for (vector<double>& opt_vec : optimization_vectors_) {
      vector<double> empty_vec(inputs.rows(), 0.0);
      optimization_vectors_.push_back(empty_vec);
      parameter_block_.push_back(&opt_vec[0]);
    }
  }

 private:
  Parameters* params_;
  ceres::Problem problem_;

  // handeled automatically
  vector<double*> parameter_block_;
  vector<vector<double>> optimization_vectors_;
};

}  // namespace optimizer

