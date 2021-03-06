// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include <memory>
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
using commons::CalculateSquaredDistance;

class ReferenceCost : public BaseCost {
 public:
  ReferenceCost() : BaseCost(), reference_() {}
  explicit ReferenceCost(const ParameterPtr& params,
                         double cost = 0.1) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_reference", cost);
  }
  virtual ~ReferenceCost() {}

  template<typename T, class M>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T dist = T(0.)) const {
    dist = CalculateSquaredDistance<T, M>(reference_.cast<T>(), trajectory);
    return Weight<T>() * dist;
  }

  void SetReference(const Matrix_t<double>& ref) {
    reference_ = ref;
  }

  Matrix_t<double> reference_;
};

typedef std::shared_ptr<ReferenceCost> ReferenceCostPtr;

}  // namespace optimizer
