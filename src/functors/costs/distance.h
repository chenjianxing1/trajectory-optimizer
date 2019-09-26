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

class ReferenceLineCost : public BaseCost {
 public:
  ReferenceLineCost() : BaseCost(), reference_line_() {}
  explicit ReferenceLineCost(const ParameterPtr& params) :
    BaseCost(params) {
      weight_ = params_->get<double>("weight_distance", 0.1);
    }
  virtual ~ReferenceLineCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T dist = T(0.)) const {
    Line<T, 2> ref_line(reference_line_.cast<T>());
    dist = CalculateDistance<T>(ref_line, trajectory);
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
