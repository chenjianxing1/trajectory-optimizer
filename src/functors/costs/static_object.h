// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <vector>
#include <functional>
#include <memory>
#include <map>
#include <utility>
#include "src/geometry/geometry.h"
#include "src/commons/parameters.h"
#include "src/commons/commons.h"
#include "src/dynamics/dynamics.h"

namespace optimizer {

using geometry::Matrix_t;
using geometry::Polygon;
using geometry::BaseGeometry;
using geometry::Line;
using geometry::Point;
using geometry::Distance;
using commons::ParameterPtr;
using commons::Parameter;
using commons::GetSquaredObjectCosts;
using commons::ObjectOutline;


class StaticObjectCost : public BaseCost {
 public:
  StaticObjectCost() : BaseCost() {}
  explicit StaticObjectCost(const ParameterPtr& params,
                            double eps = 2.0,
                            double cost = 200.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_object", cost);
      epsilon_ = params_->set<double>("epsilon", eps);
  }
  virtual ~StaticObjectCost() {}

  template<typename T, class M>
  void Evaluate(const Matrix_t<T>& trajectory,
                const Matrix_t<T>& inputs,
                Matrix_t<T>& costs,
                T cost = T(0.)) const {
    Matrix_t<T> local_costs(costs.rows(), 1);
    local_costs.setZero();
    for (const auto& obj_out : object_outlines_) {
      GetSquaredObjectCosts<T, M>(obj_out,
                                  trajectory,
                                  local_costs,
                                  T(epsilon_),
                                  params_->get<double>("dt", 0.1));
    }
    costs += Weight<T>() * local_costs;
  }

  void AddObjectOutline(const ObjectOutline& object_outline) {
    object_outlines_.push_back(object_outline);
  }

  std::vector<ObjectOutline> object_outlines_;
  double epsilon_;
};

typedef std::shared_ptr<StaticObjectCost> StaticObjectCostPtr;

}  // namespace optimizer
