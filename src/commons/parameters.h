// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#pragma once
#include <variant>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace commons {
using std::string;
using std::variant;
using std::map;
using std::vector;

// allowed variants
typedef variant<int,
                double,
                string,
                vector<double>,
                bool,
                std::map<string, double>> Variants;
/**
 * @brief Parameter class with setter and getter capabilities
 * 
 */
class Parameter {
 public:
  template<typename T>
  T set(string name, T val) {
    parameters_[name] = val;
    return val;
  }

  template<typename T>
  T get(string name, T default_val = T()) const {
    if (parameters_.count(name) == 0)
      return default_val;
    return std::get<T>(parameters_.at(name));
  }

 private:
  map<string, Variants> parameters_;
};

typedef std::shared_ptr<Parameter> ParameterPtr;
}  // namespace commons

