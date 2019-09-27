# Copyright (c) 2019 Patrick Hart

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import unittest
import numpy as np
import matplotlib.pyplot as plt
from optimizer.optimizer import \
  Optimizer, SingleTrackFunctor, JerkCost, ReferenceLineCost, InputCost, BaseFunctor, ReferenceCost
from optimizer.commons import Parameter
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
    params.set("weight_jerk", 1.)
    params.set("weight_distance", 10.)
    params.set("function_tolerance", 1e-10)
    params.set("max_num_iterations", 1000)

    initial_state = np.array([[0., 0., 0., 10.],
                              [2., 0., 0., 10.],
                              [4., 0., 0., 10.],
                              [6., 0., 0., 10.]])
    opt_vec = np.zeros(shape=(20, 2))
    ref_line = np.array([[0., 4.],
                         [1000., 4.]])
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceLineCost(params)
    ref_cost.SetReferenceLine(ref_line)
    jerk_cost = JerkCost(params)

    input_cost = InputCost(params)
    input_cost.SetLowerBound(np.array([[-0.2, -1.0]]))
    input_cost.SetUpperBound(np.array([[0.2, 1.0]]))

    # optimization problem
    functor = opt.AddFastSingleTrackFunctor(initial_state,
                                            params,
                                            [jerk_cost, ref_cost, input_cost])
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectory(initial_state, inputs, params)
    fig, ax = plt.subplots(nrows=1, ncols=2)
    ax[0].plot(trajectory[:, 0], trajectory[:, 1], marker='o')
    ax[0].axis("equal")
    ax[1].plot(inputs[:, 0], label="Steering angle", marker='o')
    ax[1].plot(inputs[:, 1], label="Acceleration", marker='o')
    ax[1].legend()
    plt.show()
    print(len(trajectory))

  @unittest.skip("...")
  def test_single_track_optimizer_reference(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("weight_jerk", 0.0001)
    params.set("weight_reference", 10.)
    params.set("function_tolerance", 1e-8)
    params.set("max_num_iterations", 1000)
    initial_state = np.array([[0., 0., 0., 10.],
                              [2., 0., 0., 10.],
                              [4., 0., 0., 10.],
                              [6., 0., 0., 10.]])

    opt_vec = np.array([[ 0.05071483,  0.03536809],
       [ 0.07181224,  0.1720348 ],
       [ 0.07224944,  0.3210886 ],
       [ 0.06002175,  0.38527624],
       [ 0.04177699,  0.33576866],
       [ 0.02210939,  0.20158473],
       [ 0.0036275 ,  0.03475681],
       [-0.0124255 , -0.11725084],
       [-0.02555558, -0.22465375],
       [-0.0356487 , -0.27707347],
       [-0.04282223, -0.27903418],
       [-0.04738487, -0.24327421],
       [-0.04980937, -0.18454833],
       [-0.050672  , -0.11534883],
       [-0.05056044, -0.04393122],
       [-0.04997732,  0.02573537],
       [-0.04926946,  0.0928394 ],
       [-0.04860033,  0.15816866],
       [-0.04797043,  0.22268795],
       [ 0.        ,  0.        ]])
    ref = np.array([
       [ 8.00023567e+00,  3.76209485e-02,  3.76120814e-02,
         1.00070736e+01],
       [ 1.00007076e+01,  1.66468441e-01,  9.10273286e-02,
         1.00414806e+01],
       [ 1.20011650e+01,  4.03685615e-01,  1.45033274e-01,
         1.01056983e+01],
       [ 1.40014082e+01,  7.42124209e-01,  1.90189304e-01,
         1.01827535e+01],
       [ 1.60013873e+01,  1.16006132e+00,  2.21823086e-01,
         1.02499073e+01],
       [ 1.80011776e+01,  1.62879312e+00,  2.38645445e-01,
         1.02902242e+01],
       [ 2.00008954e+01,  2.11821771e+00,  2.41411409e-01,
         1.02971756e+01],
       [ 2.20006311e+01,  2.60055068e+00,  2.31944116e-01,
         1.02737254e+01],
       [ 2.40004285e+01,  3.05244837e+00,  2.12534195e-01,
         1.02287947e+01],
       [ 2.60002972e+01,  3.45588458e+00,  1.85585336e-01,
         1.01733800e+01],
       [ 2.80002328e+01,  3.79812630e+00,  1.53384028e-01,
         1.01175731e+01],
       [ 3.00002312e+01,  4.07111706e+00,  1.17930290e-01,
         1.00689183e+01],
       [ 3.20002923e+01,  4.27052783e+00,  8.08175742e-02,
         1.00320086e+01],
       [ 3.40004188e+01,  4.39468604e+00,  4.31736840e-02,
         1.00089389e+01],
       [ 3.60006141e+01,  4.44354672e+00,  5.67252053e-03,
         1.00001526e+01],
       [ 3.80008794e+01,  4.41782574e+00, -3.13886733e-02,
         1.00052997e+01],
       [ 4.00012136e+01,  4.31837124e+00, -6.79673938e-02,
         1.00238676e+01],
       [ 4.20016115e+01,  4.14580445e+00, -1.04138990e-01,
         1.00555013e+01],
       [ 4.40020632e+01,  3.90041403e+00, -1.39976479e-01,
         1.01000389e+01],
       [ 4.60023138e+01,  3.61858290e+00, -1.39976479e-01,
         1.01000389e+01]])
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceCost(params)
    ref_cost.SetReference(ref)
    jerk_cost = JerkCost(params)

    # optimization problem
    functor = opt.AddFastSingleTrackFunctor(initial_state,
                                            params,
                                            [jerk_cost, ref_cost])
    opt.Solve()
    opt.Report()
    trajectory = GenerateTrajectory(initial_state, opt.Result(), params)
    plt.plot(trajectory[:, 0], trajectory[:, 1])
    plt.plot(ref[:, 0], ref[:, 1])
    plt.axis("equal")
    plt.show()
    #print(trajectory)


if __name__ == '__main__':
  unittest.main()