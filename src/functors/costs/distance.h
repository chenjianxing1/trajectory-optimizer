// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <memory>
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
using commons::CalculateSquaredDistance;

class ReferenceLineCost : public BaseCost {
 public:
  ReferenceLineCost() : BaseCost(), reference_line_() {}
  explicit ReferenceLineCost(const ParameterPtr& params,
                             double cost = 10.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_distance", cost);
  }
  virtual ~ReferenceLineCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T dist = T(0.)) const {
    Line<T, 2> ref_line(reference_line_.cast<T>());
    dist = CalculateSquaredDistance<T>(ref_line, trajectory);
    return Weight<T>() * dist;
  }

  void SetReferenceLine(const Matrix_t<double>& ref_line) {
    reference_line_ = ref_line;
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  Matrix_t<double> reference_line_;
};

typedef std::shared_ptr<ReferenceLineCost> ReferenceLineCostPtr;

}  // namespace optimizer
