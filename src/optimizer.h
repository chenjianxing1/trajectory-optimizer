
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <vector>
#include <ceres/ceres.h>
#include "src/commons/parameters.h"
#include "src/geometry/geometry.h"

namespace optimizer {
using parameters::Parameters;
using geometry::Matrix_t;
using std::vector;

class Optimizer {
 public:
  explicit Optimizer(Parameters& params) : params_(params) {}
  void AddFunctor() {}

  // each column of an Eigen Matrix will become an optimization vector
  void AddInputs(Matrix_t<double> inputs) {
    for (vector<double>& opt_vec :  optimization_vectors_) {
      vector<double> empty_vec(inputs.rows(), 0.0);
      opt_vec.push_back(empty_vec);
      parameter_block_.push_back(&opt_vec[0]);
    }
  }

 private:
  Parameters& params_;
  ceres::Problem problem_;

  // auto handling
  vector<double*> parameter_block_;
  vector<vector<double>> optimization_vectors_;
}

}  // optimizer
