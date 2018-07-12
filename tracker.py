import math
import matplotlib
import matplotlib.pyplot as plt
from car import Car
from trajectory import Trajectory


def controller(distance_error, theta_error):
    new_steering_angle = theta_error * .85 #some cool function for later
    return new_steering_angle
    
class Tracker:

    def __init__(self, velocity, orientation, trajectory, controller): 
        self.car = Car(orientation,trajectory.p[0][0],trajectory.p[0][1], velocity) #create car
        self.controller = controller
        self.trajectory = trajectory
        
    def update(self):
        distance_error, theta_error = self.trajectory.error(self.car.x,self.car.y,self.car.angle)
        # calculate error of car
        self.car.update(self.controller(distance_error, theta_error))
        #adjust for error
        self.trajectory.update(self.car.v)
        #update trajectory (constant) #speed of car and trajectory same
        return distance_error, theta_error

if __name__ == '__main__':

    #case 1
    c = Car(0., 0., 0., 1.)
    pathListx = [0]
    pathListy = [0]
    for i in range(10):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        pathListx.append(x)
        pathListy.append(y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)

    track = Tracker(1.,math.pi/2,traj,controller)
    for i in range(9):
        track.update()

     
