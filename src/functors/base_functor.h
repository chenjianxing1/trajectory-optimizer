
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/functors/costs/base_cost.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameter;
using commons::ParameterPtr;

/**
 * @brief Base functor from which all functors shall be derived
 * 
 */
class BaseFunctor {
 public:
  BaseFunctor() : opt_vec_len_(0), param_count_(0) {}
  explicit BaseFunctor(const ParameterPtr& params) :
    params_(params), opt_vec_len_(0), param_count_(0) {}
  virtual ~BaseFunctor() = default;

  template<typename T>
  Matrix_t<T> ParamsToEigen(T const* const* parameters) {
    Matrix_t<T> eigen_params(this->GetOptVecLen(),
                             this->GetParamCount());
    for (int j = 0; j < this->GetParamCount(); j++) {
      for (int i = 0; i < this->GetOptVecLen(); i++) {
        eigen_params(i, j) = parameters[j][i];
      }
    }
    return eigen_params;
  }

  /**
   * @brief Adds a cost term, such as JerkCost to the functor
   * 
   * @param cost BaseCostPtr to cost term
   */
  void AddCost(const BaseCostPtr& cost) {
    costs_.push_back(cost);
  }

  std::vector<BaseCostPtr>& GetSquaredObjectCosts() { return costs_; }

  //! Optimization length is the row length of the optimization vector
  int GetOptVecLen() const { return opt_vec_len_; }
  void SetOptVecLen(int len) { opt_vec_len_ = len; }

  //! Parameter count is the amount of different inputs (e.g. steering angle
  //  and acceleration)
  int GetParamCount() const { return param_count_; }
  void SetParamCount(int len) { param_count_ = len; }

  ParameterPtr params_;
  std::vector<BaseCostPtr> costs_;
  int opt_vec_len_;
  int param_count_;
};

typedef std::shared_ptr<BaseFunctor> BaseFunctorPtr;

}  // namespace optimizer
