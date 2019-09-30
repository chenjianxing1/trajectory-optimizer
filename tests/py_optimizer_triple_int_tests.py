# Copyright (c) 2019 Patrick Hart

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import unittest
import numpy as np
import matplotlib.pyplot as plt

from optimizer.optimizer import \
  Optimizer, SingleTrackFunctor, JerkCost, ReferenceLineCost, \
  InputCost, BaseFunctor, ReferenceCost, StaticObjectCost, SpeedCost
from optimizer.commons import Parameter, ObjectOutline
from optimizer.dynamics import GenerateTrajectoryTripleInt
from src.commons.py_commons import DrawPolygon, GetColorMap


class OptimizerTests(unittest.TestCase):
  def test_single_track_optimizer(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-8)
    params.set("max_num_iterations", 1000)

    initial_state = np.array([[0.,  1., 0., 0., 0., 0., 0., 0., 0.],
                              [0.2, 1., 0., 0., 0., 0., 0., 0., 0.],
                              [0.4, 1., 0., 0., 0., 0., 0., 0., 0.],
                              [0.6, 1., 0., 0., 0., 0., 0., 0., 0.],
                              ])
    opt_vec = np.zeros(shape=(30, 3))
    ref_line = np.array([[0., 4.],
                         [1000., 4.]])

    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceLineCost(params, 10.)
    ref_cost.SetReferenceLine(ref_line)
    jerk_cost = JerkCost(params, 1000.)


    # optimization problem
    functor = opt.AddTripleIntFunctor(initial_state,
                                      params,
                                      [jerk_cost,
                                       ref_cost])
    opt.FixOptimizationVector(0, 1)    
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectoryTripleInt(initial_state, inputs, params)


    fig, ax = plt.subplots(nrows=1, ncols=2)

    colorbar_axis = ax[0].plot(trajectory[:, 0], trajectory[:, 3])
    
    ax[0].axis("equal")
    ax[1].plot(inputs[:-1, 0], label="ax", marker='o')
    ax[1].plot(inputs[:-1, 1], label="ay", marker='o', color="green")
    ax[1].plot(inputs[:-1, 2], label="az", marker='o', color="red")
    ax[1].legend()

    plt.show()
    print(repr(trajectory))
    print(repr(inputs))

if __name__ == '__main__':
  unittest.main()