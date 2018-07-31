import cma
from neuralnetwork import NN
from tracker import Tracker
import random
from numpy.random import rand
from numpy import pi, std, array, linspace
from car import Car
from trajectory import Trajectory
#from multiprocessing import Pool
import matplotlib.pyplot as plt
from colorsys import hsv_to_rgb
import sys
import numpy as np
import os

N_TRAIN_DIRECTIONS = 50
N_TEST_DIRECTIONS = 100
N_NEURONS = 20
VELOCITY = 1

t=[[0,0],[15,50],[35,25],[50,75]]
traj = Trajectory(t)
traj_points = zip(*t)
plt.plot(traj_points[0], traj_points[1], 'b-o')
array = []
if len(sys.argv) <= 1:
    print "type python name.file or python -i name.file"

if len(sys.argv) == 2:
    f = open(sys.argv[1],"w+")
    f.write(str(t))
    f.write("\r\n")
    def objective(x, v):
        controller = NN(x).controller
        trackers = [Tracker(v, i*2*pi/N_TRAIN_DIRECTIONS, traj.copy(), controller) for i in range(N_TRAIN_DIRECTIONS)]
        costs = map(lambda x: x.run()[0], trackers)
        return reduce(lambda x, y: x+y, costs)/len(costs)
    x = 2*rand(1,4*N_NEURONS+1)[0]-1
    res = cma.fmin2(objective,
                    x,
                    .5,
                    args=(VELOCITY, ),
                    options={'popsize': 152,
                            'bounds': [-1, 1],
                            'maxiter': 100}) # 5th is mean of final sample distribution
    res=res[1].result[0]
    #res = np.array([-0.4100800238312367, 0.9065876897750335, -0.9980261306813134, -0.9625396518439127, -0.9970185407054406, -0.1557137884977462, -0.23475197711323503, 0.9623407056339532, 0.0003643043795770796, 0.9998784151854134, -0.17913962495300492, 0.7288744997616561, 0.9997863882024728, -0.4679133960037745, 0.9837599267239302, 0.5973218078556877, -0.14236785040110517, -0.19385799545882604, -0.19673596952145042, 0.9732559394524914, 0.504706866048519])
    controller = NN(res).controller
    trackers = [Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj.copy(),controller) for i in range(N_TEST_DIRECTIONS)]
    traces = map(lambda x: x.run(), trackers)
    # costs = zip(*traces)[0]
    # average = sum(costs)/len(traces)
    # print "average cost" , average
    # std = std(costs)
    # print "standard deviation" , std
    for i, trace in enumerate(traces):
        plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, N_TEST_DIRECTIONS)[i],1,1))
    plt.show()
    res = res.tolist()
    f.write(str(res))
    f.close()

elif sys.argv[1] == '-i' and len(sys.argv) == 3:
    if os.path.isfile(sys.argv[2]):
        searchfile = open(sys.argv[2])
        lines = searchfile.readlines()
        res = np.array(eval(lines[1]))
        searchfile.close()
        controller = NN(res).controller
        trackers = [Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj.copy(),controller) for i in range(N_TEST_DIRECTIONS)]
        traces = map(lambda x: x.run(), trackers)
        for i, trace in enumerate(traces):
            plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, N_TEST_DIRECTIONS)[i],1,1))
        plt.show()
    else:
        print "file does not exist"
