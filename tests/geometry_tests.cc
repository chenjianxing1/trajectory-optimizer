// Copyright (c) 2019 Patrick Hart
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "src/geometry/geometry.h"


TEST(geometry, line_test) {
  using geometry::Line_t;
  using geometry::Matrix_t;
  Matrix_t<double> start(2, 1);
  start << 0.0, 0.0;
  Matrix_t<double> end(2, 1);
  end << 1.0, 0.0;

  // line segment
  Line_t<double> line;
  line = Line_t<double>::Through(start, end);


  // distance checks
  Matrix_t<double> pt(2, 1);
  pt << -50.0, 2.0;
  double lambda = 1.0;
  std::cout << "Projection onto line: " << line.projection(pt) << std::endl;
  std::cout << "Point on line: " << line.pointAt(1) << std::endl;

  // std::cout << "Pt on line: " << line.pointAt(9) << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
