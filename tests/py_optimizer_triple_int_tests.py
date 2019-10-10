# Copyright (c) 2019 Patrick Hart

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import unittest
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from optimizer.optimizer import \
  Optimizer, SingleTrackFunctor, JerkCost, ReferenceLineCost, \
  InputCost, BaseFunctor, ReferenceCost, StaticObjectCost, SpeedCost
from optimizer.commons import Parameter, ObjectOutline, CalculateSquaredJerkTI
from optimizer.dynamics import GenerateTrajectoryTripleInt
from src.commons.py_commons import DrawPolygon, GetColorMap


class OptimizerTests(unittest.TestCase):
  def test_triple_int_optimizer(self):
    params = Parameter()
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-6)
    params.set("max_num_iterations", 1000)

    initial_state = np.array([[0.,10., 0., 0., 10., 0.,  0., 1., 0.],
                              [2., 10., 0., 2., 10., 0., 0.2, 1., 0.],
                              [4., 10., 0., 4., 10., 0., 0.4, 1., 0.],
                              [6., 10., 0., 6., 10., 0., 0.6, 1., 0.]])
    opt_vec = np.zeros(shape=(30, 3))
    ref_line = np.array([[10., -100.],
                         [50.1, 100.1]])

    
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceLineCost(params, 10.)
    ref_cost.SetReferenceLine(ref_line)
    jerk_cost = JerkCost(params, 100.)
    speed_cost = SpeedCost(params, 10.)
    speed_cost.SetDesiredSpeed(20.)

    # optimization problem
    functor = opt.AddTripleIntFunctor(initial_state,
                                      params,
                                      [ref_cost, jerk_cost, speed_cost])
    opt.FixOptimizationVector(0, 1)    
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectoryTripleInt(initial_state, inputs, params)

    fig = plt.figure()
    plt.subplot(144)
    ax = fig.add_subplot(144, projection='3d')
    plt.plot(trajectory[:, 0], trajectory[:, 3], trajectory[:, 6], marker='o')
    plt.plot([10, 50.1], [-100, 100], [0., 0.], marker='o')
    plt.axis("equal")
    plt.subplot(141)
    plt.plot(trajectory[:-3, 1], label="vx", marker='o')
    plt.plot(trajectory[:-3, 4], label="vy", marker='o', color="green")
    plt.plot(trajectory[:-3, 7], label="vz", marker='o', color="red")
    plt.xlabel("v [m/s]")
    plt.legend()
    plt.subplot(142)
    plt.plot(trajectory[:, 0], trajectory[:, 3], marker='o')
    plt.plot([10, 50.1], [-100, 100], marker='o')
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.subplot(143)
    plt.plot(trajectory[:, 3], trajectory[:, 6], marker='o')
    plt.axis("equal")
    plt.xlabel("y [m]")
    plt.ylabel("z [m]")
    # fig, ax = plt.subplots(nrows=1, ncols=2)
    # colorbar_axis = ax[0].plot(trajectory[:, 0], trajectory[:, 3])
    # ax[0].axis("equal")
    # ax[1].plot(inputs[:-1, 0], label="ax", marker='o')
    # ax[1].plot(inputs[:-1, 1], label="ay", marker='o', color="green")
    # ax[1].plot(inputs[:-1, 2], label="az", marker='o', color="red")
    # ax[1].legend()

    plt.show()
    print(repr(trajectory))
    print(repr(inputs))

if __name__ == '__main__':
  unittest.main()