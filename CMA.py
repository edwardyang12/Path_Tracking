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

N_TRAIN_DIRECTIONS = 50
N_TEST_DIRECTIONS = 100
N_NEURONS = 10
VELOCITY = 1

t=[[0,0],[50,50],[100,0],[150,50]]
traj = Trajectory(t)
traj_points = zip(*t)
plt.plot(traj_points[0], traj_points[1], 'b-o')
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
print res[1].result
res=res[1].result[0]
controller = NN(res).controller
trackers = [Tracker(VELOCITY,i*2*pi/N_TEST_DIRECTIONS,traj.copy(),controller) for i in range(N_TEST_DIRECTIONS)]
traces = map(lambda x: x.run(), trackers)
costs = zip(*traces)[0]
average = sum(costs)/len(traces)
print "average cost" , average
std = std(costs)
print "standard deviation" , std
for i, trace in enumerate(traces):
    plt.plot(trace[1], trace[2], '-', color=hsv_to_rgb(linspace(0, 1, N_TEST_DIRECTIONS)[i],1,1))
plt.show()


#each run's res array is different
# x (1,9) is best

#2 tests on (1,41) result[0]
