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
from src.commons.py_commons import DrawPolygon, GetColorMap


class OptimizerTests(unittest.TestCase):
  def test_single_track_optimizer(self):
    params = Parameter()
    params.set("wheel_base", 2.7)
    params.set("dt", 0.2)
    params.set("function_tolerance", 1e-8)
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
    ref_line = np.array([[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+01],
       [ 2.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+01],
       [ 4.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+01],
       [ 6.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+01],
       [ 8.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+01],
       [ 9.99939997e+00, -4.30463875e-03, -4.30592393e-03,
         9.99406153e+00],
       [ 1.19971306e+01, -2.03405218e-02, -1.17478308e-02,
         9.98393436e+00],
       [ 1.39923704e+01, -5.32967584e-02, -2.12840277e-02,
         9.97126125e+00],
       [ 1.59845144e+01, -1.06379431e-01, -3.19953707e-02,
         9.95734431e+00],
       [ 1.79731414e+01, -1.81067097e-01, -4.30841450e-02,
         9.94304879e+00],
       [ 1.99579824e+01, -2.77367780e-01, -5.38759931e-02,
         9.92880473e+00],
       [ 2.19388867e+01, -3.94078028e-01, -6.38232626e-02,
         9.91467222e+00],
       [ 2.39157899e+01, -5.29044532e-01, -7.25085470e-02,
         9.90044040e+00],
       [ 2.58886800e+01, -6.79428615e-01, -7.96477572e-02,
         9.88573518e+00],
       [ 2.78575651e+01, -8.41973335e-01, -8.50921200e-02,
         9.87012178e+00],
       [ 2.98224543e+01, -1.01325561e+00, -8.88112060e-02,
         9.85329539e+00],
       [ 3.17833555e+01, -1.18989285e+00, -9.08631109e-02,
         9.83511644e+00],
       [ 3.37402750e+01, -1.36869381e+00, -9.13680809e-02,
         9.81559270e+00],
       [ 3.56932177e+01, -1.54676003e+00, -9.04859136e-02,
         9.79484562e+00],
       [ 3.76421873e+01, -1.72154515e+00, -8.83971179e-02,
         9.77307098e+00],
       [ 3.95871864e+01, -1.89087947e+00, -8.52877092e-02,
         9.75050143e+00],
       [ 4.15282170e+01, -2.05296693e+00, -8.13374631e-02,
         9.72737561e+00],
       [ 4.34652803e+01, -2.20636175e+00, -7.67114427e-02,
         9.70391600e+00],
       [ 4.53983772e+01, -2.34993117e+00, -7.15546057e-02,
         9.68031568e+00],
       [ 4.73275086e+01, -2.48281085e+00, -6.59892879e-02,
         9.65673297e+00],
       [ 4.92526750e+01, -2.60435938e+00, -6.01165980e-02,
         9.63329181e+00],
       [ 5.11738771e+01, -2.71411662e+00, -5.40182610e-02,
         9.61008527e+00],
       [ 5.30911154e+01, -2.81176608e+00, -4.77585100e-02,
         9.58718020e+00],
       [ 5.50043903e+01, -2.89710162e+00, -4.13860399e-02,
         9.56462221e+00],
       [ 5.69137022e+01, -2.96999832e+00, -3.49360318e-02,
         9.54244062e+00],
       [ 5.88190514e+01, -3.03038789e+00, -2.84322483e-02,
         9.52065307e+00],
       [ 6.07204383e+01, -3.07823825e+00, -2.18892006e-02,
         9.49926943e+00],
       [ 6.26178630e+01, -3.11353783e+00, -1.53143860e-02,
         9.47829505e+00],
       [ 6.45133116e+01, -3.14256773e+00, -1.53143860e-02,
         9.47841405e+00]])

    obstacle_outline0 = np.array([[14., 1.7],
                                  [22., 1.7],
                                  [22., 4.7],
                                  [14., 4.7],
                                  [14., 1.7]])
    # optimizer
    opt = Optimizer(params)
    opt.SetOptimizationVector(opt_vec)

    # costs
    ref_cost = ReferenceCost(params, 100.)
    ref_cost.SetReference(ref_line)
    jerk_cost = JerkCost(params, 10000.)

    outline = ObjectOutline()
    obstacle_outline1 = obstacle_outline0 + np.array([[30.0, -3.]])
    outline.Add(obstacle_outline0, 0.)
    outline.Add(obstacle_outline1, 6.)

    object_cost = StaticObjectCost(params, 2.5, 10000.)
    object_cost.AddObjectOutline(outline)

    input_cost = InputCost(params, 10.)
    input_cost.SetLowerBound(np.array([[-0.2, -1.0]]))
    input_cost.SetUpperBound(np.array([[0.2, 1.0]]))

 
    # optimization problem
    functor = opt.AddFastSingleTrackFunctor(initial_state,
                                            params,
                                            [jerk_cost,
                                             object_cost,
                                             input_cost,
                                             ref_cost])

    opt.FixOptimizationVector(0, 1)    
    opt.Solve()
    opt.Report()
    inputs = opt.Result()
    trajectory = GenerateTrajectory(initial_state, inputs, params)


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
    print(repr(inputs))
    print(repr(trajectory))



if __name__ == '__main__':
  unittest.main()