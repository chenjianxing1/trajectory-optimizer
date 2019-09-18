
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

class BaseFunctor {
 public:
  BaseFunctor() : opt_vec_len_(0), param_count_(0) {}
  explicit BaseFunctor(Parameters* params) :
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

  //! getter and setter
  int GetOptVecLen() const { return opt_vec_len_; }
  void SetOptVecLen(int len) { opt_vec_len_ = len; }

  int GetParamCount() const { return param_count_; }
  void SetParamCount(int len) { param_count_ = len; }
  Parameters* GetParams() { return params_; }

 private:
  Parameters* params_;
  int opt_vec_len_;
  int param_count_;
};

}  // namespace optimizer
