import math
import matplotlib
import matplotlib.pyplot as plt

def shift(x,y,delta_x,delta_y):
    trans_x = x + delta_x
    trans_y = y + delta_y
    return trans_x, trans_y

def rotate(x,y,rot_angle):
    rotate_x = x * math.cos(rot_angle) - y * math.sin(rot_angle)
    rotate_y = x * math.sin(rot_angle) + y * math.cos(rot_angle)
    return rotate_x, rotate_y

def calc_phi(rotate_x,rotate_y):
    if rotate_x > 0 and rotate_y > 0:
        mod_angle = math.atan(rotate_y/rotate_x)
    elif rotate_x < 0 and rotate_y > 0:
        mod_angle = math.pi -  math.atan(rotate_y/-rotate_x)
    elif rotate_x < 0 and rotate_y < 0:
        mod_angle = math.pi + math.atan(-rotate_y/-rotate_x)
    elif rotate_x > 0 and rotate_y < 0:
        mod_angle = 2*math.pi - math.atan(-rotate_y/rotate_x)
    else:
        mod_angle = 0.
    return mod_angle

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

        x_error = self.x - x
        y_error = self.y - y
        distance_error = (y_error**2+x_error**2)**(0.5)
        if self.done():
            trans_x, trans_y = shift(
                self.p[self.pos][0],
                self.p[self.pos][1],
                -self.p[self.pos-1][0],
                -self.p[self.pos-1][1])

        else:
            trans_x, trans_y = shift(
            self.p[self.pos+1][0],
            self.p[self.pos+1][1],
            -self.p[self.pos][0],
            -self.p[self.pos][1])

        phi = calc_phi(trans_x, trans_y)
        if phi> theta and phi-theta > math.pi:
            theta_error = -(2*math.pi-(phi-theta))
        elif phi> theta and phi-theta <= math.pi:
            theta_error = phi-theta
        elif phi - theta >= -math.pi:
            theta_error = phi- theta
        else:
            theta_error = 2*math.pi + (phi-theta)
        return distance_error,theta_error

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
    c = Car(0., 0., 0., 1.)
    xList = [0]
    yList = [0]
#case 1
    c = Car(0., 0., 0., 1.)
    xList = [0]
    yList = [0]
    for i in range(10):
        delta_theta = math.pi/2
        x, y, _ = c.update(delta_theta)
        xList.append(x)
        yList.append(y)
    p = zip(xList, yList)
    traj = Trajectory(p)
    d = Car(0., 0., 0., 1.)

    for i in range(9):
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 1:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(delta_theta)

 #case 2
    c = Car(0., 0., 0., 1.)
    xList = [0]
    yList = [0]
    for i in range(10):
        delta_theta = math.pi/2
        x, y, _ = c.update(delta_theta)
        xList.append(x)
        yList.append(y)
    p = zip(xList, yList)
    traj = Trajectory(p)
    d = Car(0., 0., 0., .5)

    for i in range(9):
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 2:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(delta_theta)
#case 3
    c = Car(0., 0., 0., 1.)
    xList = [0]
    yList = [0]
    for i in range(10):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        xList.append(x)
        yList.append(y)
    p = zip(xList, yList)
    traj = Trajectory(p)
    d = Car(math.pi/4, 0., 0., 1.)

    for i in range(4):
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 3:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(-math.pi/2)
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 3:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(math.pi/2)
#case 4
    c = Car(math.pi, 0., 0., 1.)
    xList = [0]
    yList = [0]
    for i in range(10):
        delta_theta = 0
        x, y, _ = c.update(delta_theta)
        xList.append(x)
        yList.append(y)
    p = zip(xList, yList)
    traj = Trajectory(p)
    d = Car(math.pi/4, 0., 0., 1.)

    for i in range(4):
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 4:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(-math.pi/2)
        derr, therr = traj.error(d.x, d.y, d.angle)
        print "case 4:", "%.2f, %.2f" % (derr, therr)
        traj.update(1)
        d.update(math.pi/2)
