// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "src/commons/parameters.h"


TEST(parameters, set_and_get_parameters) {
  using parameters::Parameters;
  using std::string;
  
  Parameters params;
  params.set<double>("wheel_base", 1.7);
  params.set<int>("num", 1);
  params.set<string>("debug", "true");

  ASSERT_EQ(params.get<double>("wheel_base"), 1.7);
  ASSERT_EQ(params.get<int>("num"), 1);
  ASSERT_EQ(params.get<string>("debug"), "true");

  ASSERT_EQ(params.get<double>("wheel_base_2", 2.0), 2.0);
  ASSERT_EQ(params.get<int>("num_2", 2), 2);
  ASSERT_EQ(params.get<string>("debug_2", "no"), "no");
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}