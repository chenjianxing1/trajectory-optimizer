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
from optimizer.dynamics import GenerateTrajectory

class OptimizerTests(unittest.TestCase):
  def test_parameters(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("weight_jerk", 10e4)
    wheel_base = params.get("wheel_base", 2.9)
    self.assertEqual(wheel_base, 2.7)
    wheel_base_undefined = params.get("wheel_base_undefined", 2.9)
    self.assertEqual(wheel_base_undefined, 2.9)

  def test_single_track_optimizer(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-10)
    params.set("max_num_iterations", 1000)

    initial_state = np.array([[0., 0., 0., 10.],
                              [2., 0., 0., 10.],
                              [4., 0., 0., 10.],
                              [6., 0., 0., 10.]])
    opt_vec = np.zeros(shape=(30, 2))
    ref_line = np.array([[0., 4.],
                         [1000., 4.]])

    obstacle_outline0 = np.array([[14., 1.7],
                                  [22., 1.7],
                                  [22., 4.7],
                                  [14., 4.7],
                                  [14., 1.7]])
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceLineCost(params, 100.)
    ref_cost.SetReferenceLine(ref_line)
    jerk_cost = JerkCost(params, 10000.)

    outline = ObjectOutline()
    obstacle_outline1 = obstacle_outline0 + np.array([[30.0, 0.0]])
    outline.Add(obstacle_outline0, 0.)
    outline.Add(obstacle_outline1, 10.)

    object_cost = StaticObjectCost(params, 2.5, 2000.)
    object_cost.AddObjectOutline(outline)

    input_cost = InputCost(params, 10.)
    input_cost.SetLowerBound(np.array([[-0.2, -1.0]]))
    input_cost.SetUpperBound(np.array([[0.2, 1.0]]))

    speed_cost = SpeedCost(params, 10.)
    speed_cost.SetDesiredSpeed(10.)
    
    # optimization problem
    functor = opt.AddFastSingleTrackFunctor(initial_state,
                                            params,
                                            [jerk_cost,
                                             ref_cost,
                                             input_cost,
                                             object_cost,
                                             speed_cost])
    opt.FixOptimizationVector(0, 1)    
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectory(initial_state, inputs, params)
    fig, ax = plt.subplots(nrows=1, ncols=2)
    for t in np.arange(0, 6, 0.1):
        poly = outline.Query(t)
        ax[0].plot(poly[:, 0], poly[:, 1])

    #ax[0].plot(obstacle_outline[:, 0], obstacle_outline[:, 1])
    ax[0].plot(trajectory[:, 0], trajectory[:, 1], marker='o')
    ax[0].axis("equal")
    ax[1].plot(inputs[:-1, 0], label="Steering angle", marker='o')
    ax[1].plot(inputs[:-1, 1], label="Acceleration", marker='o')
    ax[1].legend()
    plt.show()
    print(len(trajectory))


if __name__ == '__main__':
  unittest.main()