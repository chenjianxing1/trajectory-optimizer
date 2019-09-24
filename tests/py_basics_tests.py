import unittest
import numpy as np

from optimizer.optimizer import \
  Optimizer, SingleTrackFunctor, JerkCost, ReferenceCost, InputCost, BaseFunctor
from optimizer.commons import Parameter

class OptimizerTests(unittest.TestCase):
  def test_parameters(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    wheel_base = params.get("wheel_base", 2.9)
    self.assertEqual(wheel_base, 2.7)
    wheel_base_undefined = params.get("wheel_base_undefined", 2.9)
    self.assertEqual(wheel_base_undefined, 2.9)

  def test_optimizer(self):
    params = Parameter()
    opt = Optimizer(params)
    initial_state = np.array([[0., 0., 0., 5.]])
    opt_vec = np.array([[0., 0.],
                        [0., 0.],
                        [0., 0.]])
    opt.SetOptimizationVector(opt_vec)

    functor = SingleTrackFunctor(initial_state, params)
    jerk_cost = JerkCost(params)
    functor.AddCost(jerk_cost)

    opt.AddResidualBlock(functor, 1)




if __name__ == '__main__':
  unittest.main()