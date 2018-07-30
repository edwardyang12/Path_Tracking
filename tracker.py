import math
import matplotlib
import matplotlib.pyplot as plt
from car import Car
from cmath import phase, exp
from trajectory import Trajectory


def controller(distance_error, theta_error):
    if theta_error < 0:
        distance_error = -distance_error
    elif theta_error == 0.:
        distance_error = 0
    else:
        pass
    new_steering_angle = theta_error * 0.85 + distance_error * 0.25#some cool function for later
    return new_steering_angle

class Tracker:

    def __init__(self, velocity, orientation, trajectory, controller):
        self.car = Car(orientation,trajectory.p[0][0],trajectory.p[0][1], velocity) #create car
        self.controller = controller
        self.trajectory = trajectory

    def update(self):
        distance_error, theta_error = self.trajectory.error(self.car.x,self.car.y,self.car.angle)
        steering_angle = self.controller(distance_error, theta_error)
        # calculate error of car
        self.car.update(steering_angle)
        #adjust for error
        self.trajectory.update(self.car.v)
        #update trajectory (constant) #speed of car and trajectory same
        return steering_angle, distance_error, theta_error

    def run(self):
        steering_angles = []
        d_errs = []
        theta_errs = []
        carListx = [self.trajectory.p[0][0]]
        carListy = [self.trajectory.p[0][1]]
        while not self.trajectory.done():
            steering_angle, distance_error, theta_error = self.update()
            steering_angles.append(steering_angle)
            d_errs.append(distance_error)
            theta_errs.append(theta_error)
            carListx.append(self.car.x)
            carListy.append(self.car.y)
        steering_angles.append(0.)
        steering_angles = map(lambda x: phase(exp(x*1j)),steering_angles)
        theta_errs.append(0.)
        d_errs.append(((self.car.x-self.trajectory.x)**2+(self.car.y-self.trajectory.y)**2)**0.5)
        cost = sum(
            map(lambda x: 100*x[0]**2 + 100*x[1]**2 + 10**5*x[2]**2,
                zip(steering_angles, d_errs, theta_errs))) + 1000 * d_errs[-1]**2
        self.trajectory.reset()
        return cost, carListx, carListy


if __name__ == '__main__':

    #case 1
    c = Car(math.pi/4, 0., 0., 1.)
    pathListx = [0]
    pathListy = [0]
    for i in range(5):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        pathListx.append(x)
        pathListy.append(y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)

    track = Tracker(1.,math.pi/4,traj,lambda x,y: 0)
    cost = track.run()
    print cost #not perfect bc of float error y=x


    #case 2
    c = Car(0., 0., 0., 1.)
    pathListx = [0]
    pathListy = [0]
    for i in range(5):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        pathListx.append(x)
        pathListy.append(y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)

    track = Tracker(2.,0.,traj,controller)
    cost = track.run()
    print cost #perfect tracking of y=0
    print track.car.x,track.car.y

    #case 3
    c = Car(math.pi/4, 0., 0., .75)
    pathListx = [0]
    pathListy = [0]
    for i in range(4):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        pathListx.append(x)
        pathListy.append(y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)

    track = Tracker(.75,math.pi/2,traj,controller)
    cost = track.run()
    print cost #touched line

    #case 3
    c = Car(0., 0., 0., 1.)
    pathListx = [0]
    pathListy = [0]
    for i in range(5):
        delta_theta = math.pi/2
        x, y, _ = c.update(delta_theta)
        pathListx.append(x)
        pathListy.append(y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)

    track = Tracker(1.,math.pi/2,traj,controller)
    cost = track.run()
    print cost #square ?
