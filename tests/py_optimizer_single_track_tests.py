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
    params.set("weight_jerk", 5.)
    params.set("weight_distance", 10.)
    params.set("function_tolerance", 1e-8)
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
    print(repr(trajectory))
    print(repr(inputs))

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
                              [4., 0., 0., 10.]])

    opt_vec = np.array([[ 0.        ,  0.        ],
                        [ 0.        ,  0.        ],
                        [ 0.03247779,  0.01448013],
                        [ 0.0488212 ,  0.0764439 ],
                        [ 0.05327367,  0.15928201],
                        [ 0.04958791,  0.22117624],
                        [ 0.04103882,  0.23815883],
                        [ 0.03023429,  0.20872582],
                        [ 0.01901619,  0.14630853],
                        [ 0.00854068,  0.06962228],
                        [-0.0005516 , -0.00454641],
                        [-0.00796977, -0.06491564],
                        [-0.01365509, -0.10642054],
                        [-0.01771017, -0.12905642],
                        [-0.02035473, -0.1360361 ],
                        [-0.0218883 , -0.13191998],
                        [-0.02264898, -0.12116708],
                        [-0.02296553, -0.10725383],
                        [ 0.        ,  0.        ],
                        [ 0.        ,  0.        ]])
    ref = np.array([[0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+01],
       [2.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+01],
       [4.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+01],
       [6.00009647e+00, 2.40718900e-02, 2.40695669e-02, 1.00028960e+01],
       [8.00031396e+00, 1.08501047e-01, 6.03003266e-02, 1.00181848e+01],
       [1.00005684e+01, 2.69099940e-01, 9.99344195e-02, 1.00500412e+01],
       [1.20007712e+01, 5.07134237e-01, 1.36961608e-01, 1.00942765e+01],
       [1.40008718e+01, 8.14227757e-01, 1.67737003e-01, 1.01419082e+01],
       [1.60008653e+01, 1.17634898e+00, 1.90504273e-01, 1.01836534e+01],
       [1.80007792e+01, 1.57693968e+00, 2.04871373e-01, 1.02129151e+01],
       [2.00006511e+01, 1.99923530e+00, 2.11337066e-01, 1.02268395e+01],
       [2.20005129e+01, 2.42784994e+00, 2.10919226e-01, 1.02259303e+01],
       [2.40003845e+01, 2.84972437e+00, 2.04886017e-01, 1.02129471e+01],
       [2.60002743e+01, 3.25455152e+00, 1.94565865e-01, 1.01916630e+01],
       [2.80001834e+01, 3.63479667e+00, 1.81211316e-01, 1.01658517e+01],
       [3.00001096e+01, 3.98542488e+00, 1.65902071e-01, 1.01386445e+01],
       [3.20000510e+01, 4.30344059e+00, 1.49482490e-01, 1.01122605e+01],
       [3.40000065e+01, 4.58733547e+00, 1.32534557e-01, 1.00880271e+01],
       [3.59999770e+01, 4.83653263e+00, 1.15388536e-01, 1.00665764e+01],
       [3.79999039e+01, 5.06833095e+00, 1.15388536e-01, 1.00665764e+01]])
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