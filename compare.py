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

N_TEST_DIRECTIONS = 1
VELOCITY = 1
STEP = 33
N_TEST_TRAJ = 1
N_TRAIN_TRAJ = 5

# time_List = []
# for i in range(int(100/VELOCITY)):
#     time_List.append(i)


def generate_path(STEP):
	path_list = []
	point = [0,0]
	location = 0
	path_list.append(point)
	for _ in range(3):
		angle = random.uniform(-pi/6,pi/6)
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

paths = generate_paths()

#NN
for count in range(len(sys.argv)-1):
    if os.path.isfile(sys.argv[int(count)]):
        name = sys.argv[int(count)+1]
        method = name.split('.')
        plt.figure(int(count)+1)
        searchfile = open(sys.argv[int(count)+1])
        lines = searchfile.readlines()
        res = np.array(eval(lines[0]))
        searchfile.close()
        controller = NN(res).controller
        trackers = []
        for i in range(N_TEST_DIRECTIONS):
            for traj in paths[:N_TEST_TRAJ]:
                traj = Trajectory(traj)
                trackers.append(Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj,controller))
        traces = map(lambda x: x.run(), trackers)
        for i, trace in enumerate(traces):
            plt.subplot(211)
            plt.title(method[0])
            plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, len(traces))[i],1,1))
            plt.subplot(212)
            print len(trace[1]), len(trace[3])
            #plt.plot(time_List, trace[3], 'b-')
            #plt.plot(time_List, trace[4], 'g-')
    else:
        print "file does not exist"

#PID
plt.figure(int(count)+2)
PID = PID_controller(0.04,0.02,0.009)
controller = PID.controller
trackers = []
for i in range(N_TEST_DIRECTIONS):
    for traj in paths[:N_TEST_TRAJ]:
        traj = Trajectory(traj)
        trackers.append(Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj,controller))
traces = map(lambda x: x.run(), trackers)
for i, trace in enumerate(traces):
        plt.subplot(211)
        plt.title('PID')
        plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, len(traces))[i],1,1))
        plt.subplot(212)
        # plt.plot(time_List, trace[3], 'b-')
        # plt.plot(time_List, trace[4], 'g-')

#simple
plt.figure(int(count)+3)
simple = Simple_controller()
controller = simple.controller
trackers = []
for i in range(N_TEST_DIRECTIONS):
    for traj in paths[:N_TEST_TRAJ]:
        traj = Trajectory(traj)
        trackers.append(Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj,controller))
traces = map(lambda x: x.run(), trackers)
for i, trace in enumerate(traces):
        plt.subplot(211)
        plt.title('Simple')
        plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, len(traces))[i],1,1))
        plt.subplot(212)
        # plt.plot(time_List, trace[3], 'b-')
        # plt.plot(time_List, trace[4], 'g-')

for i in range(N_TEST_TRAJ):
    xList, yList = zip(*paths[i])
    plt.figure(int(count)+1)
    plt.subplot(211)
    plt.plot(xList, yList)
    plt.figure(int(count)+2)
    plt.subplot(211)
    plt.plot(xList, yList)
    plt.figure(int(count)+3)
    plt.subplot(211)
    plt.plot(xList, yList)
plt.show()
