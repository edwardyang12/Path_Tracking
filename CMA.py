import matplotlib.pyplot as plt
import random
import sys
import numpy as np
import os
from neuralnetwork import NN
from tracker import Tracker
from numpy.random import rand
from numpy import pi, std, array, linspace, sin, cos
from car import Car
from trajectory import Trajectory
from multiprocessing import Pool
from colorsys import hsv_to_rgb
from controllers import PID_controller, Simple_controller
from cma import fmin2


N_TRAIN_DIRECTIONS = 50
N_TEST_DIRECTIONS = 100
N_NEURONS = 10
VELOCITY = 1
STEP = 33
N_TEST_TRAJ = 5
N_TRAIN_TRAJ = 5

def generate_path(STEP):
	path_list = []
	point = [0,0]
	location = 0
	path_list.append(point)
	for _ in range(20):
		angle = random.uniform(-pi/4,pi/4)
		y_val = sin(angle)*STEP + point[1]
		x_val = cos(angle)*STEP + point[0]
		point = [x_val,y_val]
		path_list.append(point)
	return path_list

def generate_paths():
	paths =[]
	total = N_TEST_TRAJ + N_TRAIN_TRAJ
	for i in range(total):
		path_list = generate_path(STEP)
		paths.append(path_list)
	return paths


if __name__ == '__main__':
    pool = Pool(processes = 100)
    paths = generate_paths()

    if len(sys.argv) <= 1:
        print "type python name.file or python -i name.file"

    if len(sys.argv) == 2:
        f = open(sys.argv[1],"w+")
        def objective(x, v):
            controller = NN(x).controller
            trackers = []
            for i in range(N_TRAIN_DIRECTIONS):
                for traj in paths[:N_TRAIN_TRAJ]:
                    traj = Trajectory(traj)
                    trackers.append(Tracker(v, i*2*pi/N_TRAIN_DIRECTIONS, traj, controller))
            costs = map(lambda x: x.run()[0], trackers)
            return reduce(lambda x, y: x+y, costs)/len(costs)
        x = 2*rand(1,4*N_NEURONS+1)[0]-1
        res = fmin2(objective,
                        x,
                        .5,
                        args=(VELOCITY, ),
                        options={'popsize': 152,
                                'bounds': [-1, 1],
                                'maxiter': 100}) # 5th is mean of final sample distribution
        res=res[1].result[0]
        #res = np.array([-0.4100800238312367, 0.9065876897750335, -0.9980261306813134, -0.9625396518439127, -0.9970185407054406, -0.1557137884977462, -0.23475197711323503, 0.9623407056339532, 0.0003643043795770796, 0.9998784151854134, -0.17913962495300492, 0.7288744997616561, 0.9997863882024728, -0.4679133960037745, 0.9837599267239302, 0.5973218078556877, -0.14236785040110517, -0.19385799545882604, -0.19673596952145042, 0.9732559394524914, 0.504706866048519])
        controller = NN(res).controller
        controller_perf = array([0,0,0])
        trackers = []
        for i in range(N_TEST_DIRECTIONS):
            for t in paths[N_TEST_TRAJ:]:
                traj = Trajectory(t)
                orientation = i*2*pi/N_TEST_DIRECTIONS
                trackers.append(Tracker(VELOCITY,orientation,traj,controller))
                cost= trackers.run()[0]
                values = array([cost,t,orientation])
                controller_perf = np.vstack((controller_perf,t,values))
        #TODO print paths
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
        f.write(str(controller_perf))
        f.close()

    elif sys.argv[1] == '-i' and len(sys.argv) == 3:
        if os.path.isfile(sys.argv[2]):
            #traj_points = zip(*t)
            #plt.plot(traj_points[0], traj_points[1], 'b-o')

            searchfile = open(sys.argv[2])
            lines = searchfile.readlines()
            res = np.array(eval(lines[1]))
            searchfile.close()
            controller = NN(res).controller
            trackers = []
            for i in range(N_TEST_DIRECTIONS):
                for traj in paths[N_TEST_TRAJ:]:
                    traj = Trajectory(traj)
                    trackers.append(Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj,controller))
            traces = map(lambda x: x.run(), trackers)
            for i, trace in enumerate(traces):
                plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, N_TEST_DIRECTIONS)[i],1,1))
            plt.show()
        else:
            print "file does not exist"
