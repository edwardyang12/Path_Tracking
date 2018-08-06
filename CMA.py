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



N_TRAIN_DIRECTIONS = 24
N_TRAIN_TRAJ = 24
N_TEST_DIRECTIONS = 6
N_NEURONS = 10
VELOCITY = 1
STEP = 33
N_TEST_TRAJ = 5

def generate_path(STEP):
	path_list = []
	point = [0,0]
	location = 0
	path_list.append(point)
	for _ in range(3):
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

def function_x(args):
    x, v, dir, traj = args
    controller = NN(x).controller
    traj = Trajectory(traj)
    tracker = Tracker(v, dir, traj, controller)
    return tracker.run()[0]


if __name__ == '__main__':
    pool = Pool()
    paths = generate_paths()

    if len(sys.argv) <= 1:
        print "type python CMA.py name.file or python CMA.py -i name.file"

    elif len(sys.argv) == 2:
        def objective(x, v):
            controller = NN(x).controller
            trackers = []
            for i in range(N_TRAIN_DIRECTIONS):
                for traj in paths[:N_TRAIN_TRAJ]:
                    trackers.append([x,v,i*2*pi/N_TRAIN_DIRECTIONS,traj])
            costs = pool.map(function_x, trackers)
            #return reduce(lambda x, y: x+y, costs)/len(costs)
            return max(costs)
        x = 2*rand(1,4*N_NEURONS+1)[0]-1
        res = fmin2(objective,
                        x,
                        .5,
                        args=(VELOCITY, ),
                        options={'popsize': 152,
                                'bounds': [-1, 1],
                                'maxiter': 200}) # 5th is mean of final sample distribution
        res=res[1].result[0]
        controller = NN(res).controller
        trackers = []
        for i in range(N_TEST_DIRECTIONS):
            for t in paths[N_TRAIN_TRAJ:]:
                traj = Trajectory(t)
                orientation = i*2*pi/N_TEST_DIRECTIONS
                trackers.append(Tracker(VELOCITY,orientation,traj,controller))
        traces = map(lambda x: x.run(), trackers)
        for i, trace in enumerate(traces):
            plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, len(traces))[i],1,1))
        plt.show()
        res = res.tolist()
        f = open(sys.argv[1],"w+")
        f.write(str(res))
        f.close()

    elif sys.argv[1] == '-i' and len(sys.argv) == 3:
        if os.path.isfile(sys.argv[2]):
            #traj_points = zip(*t)
            #plt.plot(traj_points[0], traj_points[1], 'b-o')

            searchfile = open(sys.argv[2])
            lines = searchfile.readlines()
            res = np.array(eval(lines[0]))
            searchfile.close()
            controller = NN(res).controller
            trackers = []
            for i in range(N_TEST_DIRECTIONS):
                for traj in paths[N_TRAIN_TRAJ:]:
                    traj = Trajectory(traj)
                    trackers.append(Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj,controller))
            traces = map(lambda x: x.run(), trackers)
            for i, trace in enumerate(traces):
                plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, len(traces))[i],1,1))
            plt.show()
        else:
            print "file does not exist"
