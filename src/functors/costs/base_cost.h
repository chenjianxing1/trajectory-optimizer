
// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <memory>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"

namespace optimizer {

using geometry::Matrix_t;
using commons::Parameter;
using commons::ParameterPtr;

class BaseCost {
 public:
  BaseCost() {}
  explicit BaseCost(const ParameterPtr& params) :
    params_(params),
    weight_(0.) {}
  virtual ~BaseCost() = default;

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) const { return T(0.); }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  ParameterPtr params_;
  double weight_;
};

typedef std::shared_ptr<BaseCost> BaseCostPtr;

}  // namespace optimizer
