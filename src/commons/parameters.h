#pragma once
#include <variant>
#include <string>
#include <vector>
#include <map>

namespace commons {
using std::string;
using std::variant;
using std::map;
using std::vector;

// allowed variants
typedef variant<int, double, string, vector<double>> Variants;

class Parameters {
 public:
  template<typename T>
  void set(string name, T val) {
    parameters_[name] = val;
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

}  // namespace parameters
