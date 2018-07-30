from numpy import pi, tanh, array, dot

class NN:

    def __init__(self,x):
        h = (len(x)-1)/4
        self.weight_1 = x[0:h]
        self.weight_0_d = x[h:2*h]
        self.weight_0_t = x[2*h:3*h]
        self.bias_0 = x[3*h:4*h]
        self.bias_1_0 = x[4*h]

    def controller(self,d_err,t_err):
        output = tanh(dot(self.weight_1, tanh(
            (self.weight_0_d * d_err)+(
                self.weight_0_t * t_err))+self.bias_0) + self.bias_1_0)

        return pi*(output + 1)


if __name__ == '__main__':
    from numpy.random import rand

    #logical &
    x = array([1.,1.,1.,0.,-0.25])
    neural_net = NN(x)
    d_err  = 0
    t_err = 0
    angle = neural_net.controller(d_err, t_err)
    print angle/pi-1

    #logical xor
    x = array([-1.,-1.,.25,.5,-.25,-.5,1.,-1.,.5])
    neural_net = NN(x)
    d_err  = 1
    t_err = 0
    angle = neural_net.controller(d_err, t_err)
    print angle/pi-1
