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

  template<typename T, class M>
  void Evaluate(const Matrix_t<T>& trajectory,
                const Matrix_t<T>& inputs,
                Matrix_t<T>& costs) {
    Matrix_t<T> local_costs(costs.rows(), 1);
    local_costs.setZero();
    Line<T, 2> ref_line(reference_line_.cast<T>());
    CalculateSquaredDistance<T, M>(ref_line, trajectory, local_costs);
    costs +=  Weight<T>() * local_costs;
  }

  void SetReferenceLine(const Matrix_t<double>& ref_line) {
    reference_line_ = ref_line;
  }

  Matrix_t<double> reference_line_;
};

typedef std::shared_ptr<ReferenceLineCost> ReferenceLineCostPtr;

}  // namespace optimizer
