# Copyright (c) 2019 Patrick Hart

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import unittest
import numpy as np
import matplotlib.pyplot as plt
from optimizer.commons import Parameter, ObjectOutline
from src.commons.py_commons import DrawPolygon, GetColorMap

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
    DrawPolygon(polygon0, "red")
    DrawPolygon(polygon1, "blue")
    DrawPolygon(polygon2, "green")

    outline.Add(polygon0, 0.)
    outline.Add(polygon1, 2.)
    outline.Add(polygon2, 8.)

    DrawPolygon(outline.Query(0.), "magenta")
    DrawPolygon(outline.Query(0.5), "magenta")
    DrawPolygon(outline.Query(1.), "magenta")
    DrawPolygon(outline.Query(1.5), "magenta")
    DrawPolygon(outline.Query(2.), "yellow")
    DrawPolygon(outline.Query(3.), "yellow")
    DrawPolygon(outline.Query(4.), "yellow")
    DrawPolygon(outline.Query(5.), "yellow")
    DrawPolygon(outline.Query(6.), "yellow")
    DrawPolygon(outline.Query(7.), "yellow")
    DrawPolygon(outline.Query(8.), "yellow")
    plt.show()

  def test_colormap(self):
    cmap, norm = GetColorMap(0., 10.)
    plt.plot(0., 0., marker='o', color=cmap(norm(0.)))
    plt.plot(1., 1., marker='o', color=cmap(norm(5.)))
    plt.plot(2., 2., marker='o', color=cmap(norm(10.)))
    plt.show()


if __name__ == '__main__':
  unittest.main()