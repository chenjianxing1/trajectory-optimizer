import unittest
import numpy as np
import matplotlib.pyplot as plt
from optimizer.optimizer import \
  Optimizer, SingleTrackFunctor, JerkCost, ReferenceCost, InputCost, BaseFunctor
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

  def test_optimizer(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("weight_jerk", 10.)
    params.set("weight_distance", 100.)
    params.set("function_tolerance", 1e-6)
    params.set("max_num_iterations", 1000)
    initial_state = np.array([[0., 0., 0., 10.],
                              [1., 0., 0., 10.],
                              [2., 0., 0., 10.]])

    opt_vec = np.zeros(shape=(20, 2))
    ref_line = np.array([[0., 1.],
                         [1000., 1.]])
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceCost(params)
    ref_cost.SetReferenceLine(ref_line)
    jerk_cost = JerkCost(params)

    input_cost = InputCost(params)
    input_cost.SetLowerBound(np.array([[-0.2, -1.0]]))
    input_cost.SetUpperBound(np.array([[0.2, 1.0]]))

    # optimization problem
    functor = opt.AddSingleTrackFunctor(initial_state,
                                        params,
                                        [jerk_cost, ref_cost, input_cost])
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectory(initial_state, inputs, params)
    plt.plot(trajectory[:, 0], trajectory[:, 1])
    #plt.axis('equal')
    plt.show()
    #print(trajectory)



if __name__ == '__main__':
  unittest.main()