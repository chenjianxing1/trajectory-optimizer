# Copyright (c) 2019 Patrick Hart

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import unittest
import numpy as np
import matplotlib.pyplot as plt
from optimizer.commons import Parameter, ObjectOutline

def _draw_poly(pts, color="red"):
  plt.plot(pts[:, 0], pts[:, 1], color=color)

class PyOutlineTests(unittest.TestCase):
  def test_outline(self):
    outline = ObjectOutline()
    polygon0 = np.array([[0.0, 0.0,],
                         [4.0, 0.0],
                         [4.0, 2.0],
                         [0.0, 2.0],
                         [0.0, 0.0]])
    polygon1 = polygon0 + np.array([[4.0, 2.0]])
    polygon2 = polygon1 + np.array([[8.0, -4.0]])
    _draw_poly(polygon0, "red")
    _draw_poly(polygon1, "blue")
    _draw_poly(polygon2, "green")

    outline.Add(polygon0, 0.)
    outline.Add(polygon1, 2.)
    outline.Add(polygon2, 8.)

    _draw_poly(outline.Query(0.), "magenta")
    _draw_poly(outline.Query(0.5), "magenta")
    _draw_poly(outline.Query(1.), "magenta")
    _draw_poly(outline.Query(1.5), "magenta")
    _draw_poly(outline.Query(2.), "yellow")
    _draw_poly(outline.Query(3.), "yellow")
    _draw_poly(outline.Query(4.), "yellow")
    _draw_poly(outline.Query(5.), "yellow")
    _draw_poly(outline.Query(6.), "yellow")
    _draw_poly(outline.Query(7.), "yellow")
    _draw_poly(outline.Query(8.), "yellow")
    plt.show()

if __name__ == '__main__':
  unittest.main()