
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
#include "src/functors/costs/base_cost.h"
#include "src/functors/dynamic_functor.h"

namespace optimizer {

using commons::Parameter;
using commons::ParameterPtr;
using geometry::Matrix_t;
using optimizer::BaseFunctor;
using std::vector;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::TrivialLoss;

/**
 * @brief Main optimization class
 * 
 */
class Optimizer {
 public:
  explicit Optimizer(const ParameterPtr& params) :
    optimization_vectors_(),
    parameter_block_(),
    problem_(),
    options_(),
    params_(params),
    optimization_vector_len_(0) {
      options_.max_num_consecutive_invalid_steps =
        params->get<int>("max_num_consecutive_invalid_steps", 100);
      options_.max_num_iterations =
        params->get<int>("max_num_iterations", 4000);
      options_.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
      options_.function_tolerance =
        params->get<double>("function_tolerance", 1e-8);
      options_.line_search_direction_type =
        ceres::LineSearchDirectionType::BFGS;
      options_.minimizer_progress_to_stdout =
        params->get<bool>("minimizer_progress_to_stdout", false);
  }

  /**
   * @brief Due to pointer handling a wrapper for the python bindings needs
   * to be provided
   * 
   * @tclass F Type of functor
   * @param initial_states Initial sates of the trajectory
   * @param params Parameter class
   * @param costs Cost terms (such as JerkCost, etc.)
   */
  template<class F>
  void PythonAddSingleTrackFunctor(const Matrix_t<double>& initial_states,
                                   const ParameterPtr& params,
                                   const std::vector<BaseCostPtr>& costs) {
    BaseFunctor* functor =
      new F(initial_states, params);
    for (auto& cost : costs) {
      functor->AddCost(cost);
    }
    this->AddResidualBlock<F>(functor);
  }

  /**
   * @brief Adds a residual block to the optimization problem
   * 
   * @tclass F The functor that shall be used, such as the DynamicFunctor 
   * @tparam 4 Stride used for the AutoDiff
   * @param functor Pointer to the used functor
   * @param num_residuals amount of residuals within the functor
   */
  template<class F, int N = 4>
  void AddResidualBlock(BaseFunctor* functor, int num_residuals = 1) {
    DynamicAutoDiffCostFunction<F, N>* ceres_functor =
      new DynamicAutoDiffCostFunction<F, N>(dynamic_cast<F*>(functor));
    for (vector<double>& vec : optimization_vectors_) {
      ceres_functor->AddParameterBlock(optimization_vector_len_);
    }
    functor->SetOptVecLen(optimization_vector_len_);
    functor->SetParamCount(parameter_block_.size());
    ceres_functor->SetNumResiduals(num_residuals);
    problem_.AddResidualBlock(ceres_functor,
                              new ceres::TrivialLoss(),
                              parameter_block_);
  }

  /**
   * @brief Set the Optimization Vector object
   * 
   * @param inputs 
   */
  void SetOptimizationVector(const Matrix_t<double>& inputs) {
    for (int i = 0; i < inputs.cols(); i++) {
      vector<double> empty_vec;
      for (int j = 0; j < inputs.rows(); j++) {
        empty_vec.push_back(inputs(j, i));
      }
      optimization_vectors_.push_back(empty_vec);
      parameter_block_.push_back(&optimization_vectors_[i][0]);
    }
    optimization_vector_len_ = inputs.rows();
  }
  
  /**
   * @brief Fix the optimization vector within a certain range
   * 
   * @param start Starting index
   * @param end Ending index
   */
  void FixOptimizationVector(int start, int end) {
    vector<int> vec;
    for (int i = start; i < end; i++)
      vec.push_back(i);
    for ( auto& pblock : parameter_block_)
      problem_.SetParameterization(
        pblock,
        new ceres::SubsetParameterization(optimization_vector_len_, vec));
  }

  /**
   * @brief Solves the formulated optimization problem
   * 
   */
  void Solve() {
    ceres::Solve(options_, &problem_, &summary_);
  }

  /**
   * @brief Returns the optimized optimization vector
   * 
   * @return Matrix_t<double> Inputs for the dynamic model
   */
  Matrix_t<double> Result() {
    Matrix_t<double> result(optimization_vector_len_,
                            parameter_block_.size());
    for ( int i = 0; i < optimization_vector_len_; i++ ) {
      for ( int j = 0; j < parameter_block_.size(); j++ ) {
        result(i, j) = parameter_block_[j][i];
      }
    }
    return result;
  }

  /**
   * @brief Information about the optimization process
   * 
   */
  void Report() {
    std::cout << summary_.FullReport() << std::endl;
  }

 private:
  ParameterPtr params_;
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  // handeled automatically
  vector<double*> parameter_block_;
  vector<vector<double>> optimization_vectors_;
  int optimization_vector_len_;
};

}  // namespace optimizer

