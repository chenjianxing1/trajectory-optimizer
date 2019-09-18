
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

using commons::Parameters;
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
    problem_(),
    options_(),
    params_(params),
    optimization_vector_len_(0) {
      options_.max_num_consecutive_invalid_steps =
        params->get<int>("max_num_consecutive_invalid_steps", 200);
      options_.max_num_iterations =
        params->get<int>("max_num_iterations", 4000);
      options_.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
      options_.function_tolerance =
        params->get<double>("function_tolerance", 1e-12);
      options_.line_search_direction_type =
        ceres::LineSearchDirectionType::LBFGS;
      options_.minimizer_progress_to_stdout =
        params->get<bool>("minimizer_progress_to_stdout", false);
  }

  template<typename T, int N = 4>
  void AddResidualBlock(
    BaseFunctor* functor,
    LossFunction* loss = new TrivialLoss(),
    int num_residuals = 1) {
    DynamicAutoDiffCostFunction<T, N>* ceres_functor =
      new DynamicAutoDiffCostFunction<T, N>(dynamic_cast<T*>(functor));
    for (vector<double>& vec : optimization_vectors_) {
      ceres_functor->AddParameterBlock(optimization_vector_len_);
      // std::cout << "Parameterblock size: " <<  parameter_block_.size() << std::endl;
      // std::cout << "Opt size: " <<  vec.size() << std::endl;
    }
    functor->SetOptVecLen(optimization_vector_len_);
    functor->SetParamCount(parameter_block_.size());
    ceres_functor->SetNumResiduals(num_residuals);
    problem_.AddResidualBlock(ceres_functor, loss, parameter_block_);
  }

  // each column of an Eigen Matrix will become an optimization vector
  void SetOptimizationVector(Matrix_t<double> inputs) {
    for (int i = 0; i < inputs.cols(); i++) {
      vector<double>* empty_vec = new vector<double>(inputs.rows(), 0.0);
      optimization_vectors_.push_back(*empty_vec);
      parameter_block_.push_back(&empty_vec->at(0));
    }
    optimization_vector_len_ = inputs.rows();
  }


  void FixOptimizationVector(int start, int end) {
    vector<int> vec;
    for (int i = start; i < end; i++)
      vec.push_back(i);
    for ( auto& pblock : parameter_block_)
      problem_.SetParameterization(
        pblock,
        new ceres::SubsetParameterization(optimization_vector_len_, vec));
  }


  void Solve() {
    ceres::Solve(options_, &problem_, &summary_);
  }

  Matrix_t<double> GetOptimizationVector() {
    Matrix_t<double> result(optimization_vector_len_,
                            parameter_block_.size());

    for ( int i = 0; i < optimization_vector_len_; i++ ) {
      for ( int j = 0; j < parameter_block_.size(); j++ ) {
        result(i, j) = parameter_block_[j][i];
      }
    }
    return result;
  }

  void Report() {
    std::cout << summary_.FullReport() << std::endl;
  }

 private:
  Parameters* params_;
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;


  // handeled automatically
  vector<double*> parameter_block_;
  vector<vector<double>> optimization_vectors_;
  int optimization_vector_len_;
};

}  // namespace optimizer

