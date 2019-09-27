// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::BaseGeometry;
using geometry::Line;
using commons::ParameterPtr;
using commons::Parameter;
using commons::CalculateDistance;

class ReferenceCost : public BaseCost {
 public:
  ReferenceCost() : BaseCost(), reference_() {}
  explicit ReferenceCost(const ParameterPtr& params) :
    BaseCost(params) {
      weight_ = params_->get<double>("weight_reference", 0.1);
  }
  virtual ~ReferenceCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T dist = T(0.)) const {
    dist = CalculateDistance<T>(reference_.cast<T>(), trajectory);
    return Weight<T>() * dist;
  }

  void SetReference(const Matrix_t<double>& ref) {
    reference_ = ref;
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  Matrix_t<double> reference_;
};

typedef std::shared_ptr<ReferenceCost> ReferenceCostPtr;

}  // namespace optimizer
