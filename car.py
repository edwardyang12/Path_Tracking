import math
import matplotlib
import matplotlib.pyplot as plt

#location of car

class Car:


    def __init__(self,angle,x,y,v):
        self.angle = angle
        self.x = x
        self.y = y
        self.v = v

    def update(self, theta):
        self.angle += theta
        if self.angle > math.pi*2:
            self.angle -= 2*math.pi
        xvel = self.v * math.cos(self.angle)
        yvel = self.v * math.sin(self.angle)
        self.x = xvel + self.x
        self.y = yvel + self.y

if __name__ == '__main__':
    theta_init = 0
    x_init = 0
    y_init = 0
    v_init = 1.
    xList = [x_init]
    yList = [y_init]
    dis = Car(theta_init, x_init, y_init, v_init)
    for i in range(10):
        delta_theta = 0
        x, y, _ = dis.update(delta_theta)
        xList.append(x)
        yList.append(y)

    plt.plot(xList, yList, '-o')
    plt.show()
