import cma
from neuralnetwork import NN
from tracker import Tracker
import random
from numpy.random import rand
from numpy import pi, std
from car import Car
from trajectory import Trajectory
import matplotlib.pyplot as plt

t = [[0,0],[5,5]]
traj = Trajectory(t)
traj_points = zip(*t)
plt.plot(traj_points[0], traj_points[1], 'b-o')
def objective(x, v, theta):
    controller = NN(x).controller
    tracker = Tracker(v, theta, traj.copy(), controller)
    cost, _, _ = tracker.run()
    return cost
x = 2*rand(1,41)[0]-1
res = cma.fmin2(objective,
                x,
                .5,
                args=(.01, random.random()*2*pi),
                options={'popsize': 152,
                        'bounds': [-1, 1],
                        'maxiter': 200}) # 5th is mean of final sample distribution
print res[1].result
res=res[1].result[5]
controller = NN(res).controller
trackers = [Tracker(.005,random.random()*2*pi,traj.copy(),controller) for _ in range(100)]
traces = map(lambda x: x.run(), trackers)
costs = zip(*traces)[0]
average = sum(costs)/len(traces)
print "average cost" , average
std = std(costs)
print "standard deviation" , std
for _,xList, yList in traces:
    plt.plot(xList, yList, 'r-o')
plt.show()
