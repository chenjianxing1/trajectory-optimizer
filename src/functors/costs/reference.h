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
using commons::Parameters;
using commons::CalculateDistance;

class ReferenceCost : public BaseCost {
 public:
  ReferenceCost() : BaseCost(nullptr) {}
  explicit ReferenceCost(Parameters* params) :
    BaseCost(params) {}
  virtual ~ReferenceCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs) {
    std::cout << "we got here" << std::endl;
    Line<T, 2> ref_line(reference_line_.cast<T>());
    T dist = CalculateDistance<T>(ref_line, trajectory);
    return T(params_->get<double>("weight_distance", 0.1)) * dist * dist;
  }

  void SetReferenceLine(const Matrix_t<double>& ref_line) {
    reference_line_ = ref_line;
  }

  Matrix_t<double> reference_line_;
};

}  // namespace optimizer
