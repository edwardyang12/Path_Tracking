import math
import matplotlib
import matplotlib.pyplot as plt
from car import Car
from cmath import phase, exp
from trajectory import Trajectory
from neuralnetwork import NN
import numpy as np
from controllers import PID_controller, Simple_controller

# def controller(distance_error, theta_error):
#     if theta_error < 0:
#         distance_error = -distance_error
#     elif theta_error == 0.:
#         distance_error = 0
#     else:
#         pass
#     new_steering_angle = theta_error * 0.85 + distance_error * 0.25#some cool function for later
#     return new_steering_angle
#
# def PID_controller(distance_error, theta_error):
#     accumulated_error = 0
#     list_Theta_error.append(theta_error)
#     if len(list_Theta_error) <10:
#         for i in range(len(list_Theta_error)):
#             accumulated_error = list_Theta_error[i] + accumulated_error
#     else:
#         for i in range(len(list_Theta_error)-10, len(list_Theta_error)):
#             accumulated_error = list_Theta_error[i] + accumulated_error
#     previous_error = list_Theta_error[len(list_Theta_error)-2]
#     new_steering_angle = 5.0*theta_error + 0.0025 *accumulated_error + 0.001*(theta_error-previous_error)/1
#     return new_steering_angle

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
        # cost = sum(
        #     map(lambda x: 100*x[0]**2 + 100*x[1]**2 + 10**5*x[2]**2,
        #         zip(steering_angles, d_errs, theta_errs))) + 1000 * d_errs[-1]**2
        cost = sum(map(lambda x: 100*x**2, d_errs[:-1]))+10000*d_errs[-1]**2
        self.trajectory.reset()
        return cost, carListx, carListy, d_errs, theta_errs


if __name__ == '__main__':
    list_Theta_error = []
    #case 1
    c = Car(math.pi/4, 0., 0., 1.)
    pathListx = [0]
    pathListy = [0]
    for i in range(5):
        delta_theta = 0
        c.update(delta_theta)
        pathListx.append(c.x)
        pathListy.append(c.y)
    p = zip(pathListx, pathListy)
    traj = Trajectory(p)
    x = np.array([0.8645556591802002, 0.8413252153328036, 0.5093569347642122, -0.7731511474785859, 0.6640466051330841, 0.17779438673458414, 0.5944740056874855, 0.7767031192816762, -0.2546650961059953, 0.8178925471533137, -0.7319540491533765, 0.48852801012873526, -0.6020099556309746, -0.6669614603724152, -0.7087107252348928, -0.5001242129876614, 0.5284651406041465, 0.6690773543233818, 0.9042094105903331, -0.6074545586458633, 0.3957572208206551, -0.5973371330983586, 0.712023323554019, 0.18599730764515482, -0.534980479737322, 0.9463750290535982, -0.9986357866668286, 0.9292588057490306, 0.35522915527662136, -0.9692544277966347, -0.15884800984321323, 0.584297294209831, -0.9133543885316171, -0.374065089777752, -0.38915751849860925, -0.9989446884221987, 0.043836563887899145, 0.9999452185418479, 0.6070615529635152, -0.8418091583257694, 0.4202498209051319, 0.5564186113435203, 0.555311886669758, -0.832890394661856, -0.98009859172038, -0.39445237156212176, -0.037779768457022134, -0.012204472378917697, 0.03488907526535291, 0.7708496093822486, -0.8784655950195925, 0.8746477155355499, -0.8575269090735795, -0.29599438495441377, -0.6668498658177002, -0.6385950413298903, 0.9697562635185897, 0.8786459903244533, 0.6482911089121831, -0.11993920927832091, -0.772720183218961, 0.17790750188249205, -0.18653760475670889, 0.3960711998381077, -0.0033684132198099626, -0.21067234485329456, -0.3753479446358803, 0.055153910155083516, -0.16949742782793037, -0.9052446924824077, 0.380809384299709, -0.8306618551590639, -0.2029395897397982, 0.6109089072465596, 0.9753912435095556, 0.9997622353974134, 0.1909207578297727, -0.6091875451850151, -0.7803408332873301, -0.004608696712309346, -0.19606055624248586])
    NN_controller = NN(x).controller
    #simple = Simple_controller().controller
    track = Tracker(1.,0,traj,NN_controller)
    plt.plot(pathListx,pathListy,'-')
    cost,carListx,carListy = track.run()
    plt.plot(carListx, carListy, '-')
    plt.show()
    print cost #not perfect bc of float error y=x
    #
    # #case 2
    # c = Car(0., 0., 0., 1.)
    # pathListx = [0]
    # pathListy = [0]
    # for i in range(5):
    #     delta_theta = 0
    #     x, y, _ = c.update(delta_theta)
    #     pathListx.append(x)
    #     pathListy.append(y)
    # p = zip(pathListx, pathListy)
    # traj = Trajectory(p)
    #
    # track = Tracker(2.,0.,traj,controller)
    # cost = track.run()
    # print cost #perfect tracking of y=0
    # print track.car.x,track.car.y
    #
    # #case 3
    # c = Car(math.pi/4, 0., 0., .75)
    # pathListx = [0]
    # pathListy = [0]
    # for i in range(4):
    #     delta_theta = 0
    #     x, y, _ = c.update(delta_theta)
    #     pathListx.append(x)
    #     pathListy.append(y)
    # p = zip(pathListx, pathListy)
    # traj = Trajectory(p)
    #
    # track = Tracker(.75,math.pi/2,traj,controller)
    # cost = track.run()
    # print cost #touched line
    #
    # #case 3
    # c = Car(0., 0., 0., 1.)
    # pathListx = [0]
    # pathListy = [0]
    # for i in range(5):
    #     delta_theta = math.pi/2
    #     x, y, _ = c.update(delta_theta)
    #     pathListx.append(x)
    #     pathListy.append(y)
    # p = zip(pathListx, pathListy)
    # traj = Trajectory(p)
    #
    # track = Tracker(1.,math.pi/2,traj,controller)
    # cost = track.run()
    # print cost #square ?
