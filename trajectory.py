import math
import matplotlib
import matplotlib.pyplot as plt
from numpy import array, dot
from math import acos
from cmath import phase, exp
from numpy.linalg import norm


class Trajectory:
    def __init__(self, p):
        self.p = p
        self.reset()

    def update(self, d):
        assert d >= 0
        if self.pos >= len(self.p)-1:
             return self.x, self.y
        xDis = self.p[self.pos+1][0] - self.x
        yDis = self.p[self.pos+1][1] - self.y
        Dis = (xDis**2+yDis**2)**(0.5)
        if d >= Dis:
            d = d - Dis
            self.x, self.y = self.p[self.pos+1]
            self.pos += 1
            return self.update(d)
        else:
            frac = d/Dis
            self.x = xDis * frac + self.x
            self.y = yDis * frac + self.y
            return self.x, self.y

    def error(self, x, y, theta):
        min_distance_error=float('inf')
        min_waypoint_distance=float('inf')
        pos=None
        for i in range(len(self.p)):
            if i==0: continue
            q=array([self.p[i][0]-self.p[i-1][0], self.p[i][1]-self.p[i-1][1]])
            c=array([x-self.p[i-1][0], y-self.p[i-1][1]])
            waypoint_distance = norm(c)
            if waypoint_distance==0:
                min_distance_error=0
                pos = i
                break
            proj_q_c = (dot(q,c)/norm(q)) * (q/norm(q))
            distance_error = norm(c-proj_q_c)
            if waypoint_distance < min_waypoint_distance:
                min_distance_error = distance_error
                min_waypoint_distance = waypoint_distance
                pos=i

        q=self.p[pos][0]-self.p[pos-1][0] + 1j*(self.p[pos][1]-self.p[pos-1][1])
        c=x-self.p[pos-1][0] + 1j*(y-self.p[pos-1][1])
        orientation_vector=exp(1j*theta)

        if c == 0 or (q/c).imag < 0:
            min_distance_error *= -1
        theta_error = phase(q/orientation_vector)

        return min_distance_error,theta_error

    def done(self):
        return self.pos >= len(self.p)-1

    def reset(self):
        self.x = self.p[0][0]
        self.y = self.p[0][1]
        self.pos = 0

    def copy(self):
        return Trajectory(self.p)

if __name__ == '__main__':
    from car import Car

#case 1
    c = Car(0., 0., 0., 1.)
    xList = []
    yList = []
    for i in range(10):
        delta_theta = math.pi/2
        c.update(delta_theta)
        xList.append(c.x)
        yList.append(c.y)
    p = zip(xList, yList)
    traj = Trajectory(p)
    d = Car(0., 0., 0., 1.)

    for i in range(10):
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 1:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(delta_theta)
#
#  #case 2
#     c = Car(0., 0., 0., 1.)
#     xList = [0]
#     yList = [0]
#     for i in range(10):
#         delta_theta = math.pi/2
#         c.update(delta_theta)
#         xList.append(c.x)
#         yList.append(c.y)
#     p = zip(xList, yList)
#     traj = Trajectory(p)
#     d = Car(0., 0., 0., .5)
#
#     for i in range(10):
#         derr, therr = traj.error(d.x, d.y, d.angle)
#         print "case 2:", "%.2f, %.2f" % (derr, therr)
#         traj.update(1)
#         d.update(delta_theta)
# #case 3
#     c = Car(0., 0., 0., 1.)
#     xList = [0]
#     yList = [0]
#     for i in range(10):
#         delta_theta = 0
#         c.update(delta_theta)
#         xList.append(c.x)
#         yList.append(c.y)
#     p = zip(xList, yList)
#     traj = Trajectory(p)
#     d = Car(math.pi/4, 0., 0., 1.)
#
#     for i in range(5):
#         derr, therr = traj.error(d.x, d.y, d.angle)
#         print "case 3:", "%.2f, %.2f" % (derr, therr)
#         traj.update(1)
#         d.update(-math.pi/2)
#         derr, therr = traj.error(d.x, d.y, d.angle)
#         print "case 3:", "%.2f, %.2f" % (derr, therr)
#         traj.update(1)
#         d.update(math.pi/2)
# #case 4
#     c = Car(math.pi, 0., 0., 1.)
#     xList = [0]
#     yList = [0]
#     for i in range(10):
#         delta_theta = 0
#         c.update(delta_theta)
#         xList.append(c.x)
#         yList.append(c.y)
#     p = zip(xList, yList)
#     traj = Trajectory(p)
#     d = Car(math.pi/4, 0., 0., 1.)
#
#     for i in range(5):
#         derr, therr = traj.error(d.x, d.y, d.angle)
#         print "case 4:", "%.2f, %.2f" % (derr, therr)
#         traj.update(1)
#         d.update(-math.pi/2)
#         derr, therr = traj.error(d.x, d.y, d.angle)
#         print "case 4:", "%.2f, %.2f" % (derr, therr)
#         traj.update(1)
#         d.update(math.pi/2)
