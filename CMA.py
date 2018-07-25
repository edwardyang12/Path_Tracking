import cma
from neuralnetwork import NN
from tracker import Tracker
import random
from numpy.random import rand
from numpy import pi, std
from car import Car
from trajectory import Trajectory
import matplotlib.pyplot as plt

t = [[0,0],[1,1]]
traj = Trajectory(t)
traj_points = [[0,1],[0,1]]
plt.plot(traj_points[0], traj_points[1], 'b-o')
def objective(x):
    controller = NN(x).controller
    tracker = Tracker(.01,random.random()*2*pi,traj, controller)
    cost, _, _ = tracker.run()
    return cost
x = rand(1,161)[0]
res = cma.fmin2(objective, x, 0.5)[0]
controller = NN(res).controller
trackers = [Tracker(.01,random.random()*2*pi,traj,controller) for _ in range(100)]
traces = map(lambda x: x.run(), trackers)
costs = zip(*traces)[0]
average = sum(costs)/len(traces)
#print average
std = std(costs)
#print std
# print the average cost and std
# plot on one graph the trajectory in blue
# and all of the traces in red
#plt.plot(carListx, carListy, 'r-o')
for _,xList, yList in traces:
    plt.plot(xList, yList, 'r-o')
plt.show()
