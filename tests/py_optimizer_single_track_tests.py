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
from optimizer.dynamics import GenerateTrajectorySingleTrack
from src.commons.py_commons import DrawPolygon, GetColorMap


class OptimizerTests(unittest.TestCase):
  def test_single_track_optimizer(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-6)
    params.set("max_num_iterations", 1000)
    params.set("num_threads", 4)

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
    obstacle_outline1 = obstacle_outline0 + np.array([[30.0, -2.]])
    outline.Add(obstacle_outline0, 0.)
    outline.Add(obstacle_outline1, 6.)

    object_cost = StaticObjectCost(params, 2.5, 10000.)
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
    trajectory = GenerateTrajectorySingleTrack(initial_state, inputs, params)

    cmap, norm = GetColorMap(0., 6.)
    fig, ax = plt.subplots(nrows=1, ncols=2)
    for t in np.arange(0, 6, 0.5):
        poly = outline.Query(t)
        ax[0].plot(poly[:, 0], poly[:, 1], color=cmap(norm(t)))

    colorbar_axis = ax[0].plot(trajectory[:, 0], trajectory[:, 1])
    for i, pt in enumerate(trajectory[:]):
        ax[0].plot(pt[0], pt[1], color=cmap(norm(params.set("dt", 0.2)*i)), marker='o')
    
    ax[0].axis("equal")
    ax[1].plot(inputs[:-1, 0], label="Steering angle", marker='o')
    ax[1].plot(inputs[:-1, 1], label="Acceleration", marker='o', color="green")
    ax[1].legend()

    plt.show()
    print(repr(trajectory))
    print(repr(inputs))

  def test_warm_start(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-6)
    params.set("max_num_iterations", 1000)

    initial_state = np.array([[0., 0., 0., 10.],
                              [2., 0., 0., 10.],
                              [4., 0., 0., 10.],
                              [6., 0., 0., 10.]])
    opt_vec = np.array([[ 0.        ,  0.        ],
                        [-0.00581466, -0.02969236],
                        [-0.0100573 , -0.05063586],
                        [-0.01290205, -0.06336555],
                        [-0.0145111 , -0.06958468],
                        [-0.01504364, -0.07147759],
                        [-0.01466189, -0.0712203 ],
                        [-0.01353391, -0.07066254],
                        [-0.01183398, -0.0711591 ],
                        [-0.00974178, -0.0735261 ],
                        [-0.00744058, -0.07806701],
                        [-0.00509113, -0.08413195],
                        [-0.0028139 , -0.09089476],
                        [-0.00069383, -0.0976187 ],
                        [ 0.00121458, -0.10373539],
                        [ 0.00288213, -0.10887319],
                        [ 0.00430011, -0.11284776],
                        [ 0.00547573, -0.11562908],
                        [ 0.00642782, -0.11729807],
                        [ 0.00718276, -0.11800159],
                        [ 0.0077706 , -0.11791354],
                        [ 0.00821974, -0.11720583],
                        [ 0.00855625, -0.11603267],
                        [ 0.0088038 , -0.11452536],
                        [ 0.0089836 , -0.11278997],
                        [ 0.00911419, -0.11090794],
                        [ 0.00921137, -0.10893774],
                        [ 0.00928801, -0.10691821],
                        [ 0.00935393, -0.10487191],
                        [ 0.        ,  0.00059502]])
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
    obstacle_outline1 = obstacle_outline0 + np.array([[30.0, -2.]])
    outline.Add(obstacle_outline0, 0.)
    outline.Add(obstacle_outline1, 6.)

    object_cost = StaticObjectCost(params, 2.5, 10000.)
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

if __name__ == '__main__':
  unittest.main()