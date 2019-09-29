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

typedef std::pair<double, Matrix_t<double>> TimedPolygonOutline;

inline Point_t<double, 2> InterpolateMatrices(const Matrix_t<double>& p0,
                                              const Matrix_t<double>&p1,
                                              double lambda) {
  return (1. - lambda)*p0 + lambda*p1;
}

inline Matrix_t<double> InterpolateTimedPolygon(const TimedPolygonOutline& p0,
                                                const TimedPolygonOutline& p1,
                                                double time) const {
  double lambda = time - p0.first / (p1.first - p0.first);
  return InterpolateMatrices(p0.second, p1.second, lambda);
}

// TODO(@hart): write test
class ObjectOutline {
 public:
  ObjectOutline() {}

  ObjectOutline(const Matrix_t<double>& outline,
                double timestamp = 0) {
    this->Add(outline, timestamp);
  }

  void Add(const Matrix_t<double>& outline, double timestamp = 0) {
    TimedPolygonOutline outline = std::make_pair(outline, timestamp),
    object_outlines_.push_back(outline);
  }

  Matrix_t<double>& Query(double timestamp_query) const {
    for (int i = 0; i < object_outlines_.size() - 1; i++) {
      if (object_outlines_[i].first < timestamp_query && \
          object_outlines_[i+1].first > timestamp_query) {
        return InterpolateTimedPolygon(object_outlines_[i],
                                       object_outlines_[i+1],
                                       timestamp_query);
      }
    }
  }

 private:
  std::vector<TimedPolygonOutline> object_outlines_;
};

class StaticObjectCost : public BaseCost {
 public:
  StaticObjectCost() : BaseCost() {}
  explicit StaticObjectCost(const ParameterPtr& params,
                            double eps = 2.0,
                            double cost = 200.) :
    BaseCost(params) {
      weight_ = params_->set<double>("weight_object", cost);
      params_->set<double>("epsilon", eps);
  }
  virtual ~StaticObjectCost() {}

  template<typename T>
  T Evaluate(const Matrix_t<T>& trajectory,
             const Matrix_t<T>& inputs,
             T cost = T(0.)) const {
    for (auto& obj_out : object_outlines_) {
      // Polygon<T, 2> obj(obj_out.cast<T>());
      cost = GetSquaredObjectCosts<T>(obj_out, trajectory, params_);
    }
    return Weight<T>() * cost;
  }

  void AddObjectOutline(const ObjectOutline& object_outline) {
    object_outlines_.push_back(object_outline);
  }

  template<typename T>
  T Weight() const {
    return T(weight_);
  }

  std::vector<ObjectOutline> object_outlines_;
};

typedef std::shared_ptr<StaticObjectCost> StaticObjectCostPtr;

}  // namespace optimizer
