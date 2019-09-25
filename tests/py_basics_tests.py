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
    initial_state = np.array([[0., 0., 0., 5.]])
    opt_vec = np.zeros(shape=(10, 2))
    ref_line = np.array([[0., 0.],
                         [5., .5],
                         [10., .5]])
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
    print(opt.Result())



if __name__ == '__main__':
  unittest.main()