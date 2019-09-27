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
using geometry::Polygon;
using geometry::BaseGeometry;
using geometry::Line;
using geometry::Distance;
using commons::ParameterPtr;
using commons::Parameter;
using commons::CalculateDistance;

class StaticObjectCost : public BaseCost {
 public:
  StaticObjectCost() : BaseCost() {}
  explicit StaticObjectCost(const ParameterPtr& params, double eps = 1.0) :
    BaseCost(params),
    epsilon_(eps) {
      weight_ = params_->get<double>("weight_object", 0.1);
  }
  virtual ~StaticObjectCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T dist = T(0.)) const {
    for (auto& obj_out : object_outlines_) {
      Polygon<T, 2> obj(obj_out.cast<T>());
      T distance = CalculateDistance<T>(obj, trajectory, T(epsilon_));
    }
    return Weight<T>() * dist;
  }

  void AddObject(const Matrix_t<double>& object_outline) {
    object_outlines_.push_back(object_outline);
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  std::vector<Matrix_t<double>> object_outlines_;
  double epsilon_;
};

typedef std::shared_ptr<StaticObjectCost> StaticObjectCostPtr;

}  // namespace optimizer
