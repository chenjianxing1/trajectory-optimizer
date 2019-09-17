// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/optimizer.h"


TEST(optimizer, basic_test) {
  using parameters::Parameters;
  using optimizer::Optimizer;
  using std::string;

  Parameters params;
  params.set<double>("wheel_base", 1.7);

  Optimizer opt(&params);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}