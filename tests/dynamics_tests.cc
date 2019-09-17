// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "src/commons/parameters.h"
#include "src/dynamics/dynamics.h"


TEST(dynamics, single_track_model) {
  using dynamics::SingleTrackState;
  using geometry::Matrix_t;

  //! add objects to world
  Matrix_t<double>* dynamic_state = new Matrix_t<double>(4, 1);
  (*dynamic_state) << 0.0, 0.0, 0.0, 5.0;  // x, y, theta, v
  SingleTrackState* single_track_state = new SingleTrackState(dynamic_state);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}