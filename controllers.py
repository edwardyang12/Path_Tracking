
class PID_controller:

    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def controller(self,distance_error, theta_error):
        accumulated_error = 0
        list_Theta_error.append(theta_error)
        if len(list_Theta_error) <10:
            for i in range(len(list_Theta_error)):
                accumulated_error = list_Theta_error[i] + accumulated_error
        else:
            for i in range(len(list_Theta_error)-10, len(list_Theta_error)):
                accumulated_error = list_Theta_error[i] + accumulated_error
        previous_error = list_Theta_error[len(list_Theta_error)-2]
        new_steering_angle = self.Kp*theta_error + self.Ki *accumulated_error + self.Kd*(theta_error-previous_error)/1
        return new_steering_angle

class Simple_controller:

    def __init__(self):
        pass

    def controller(self,distance_error, theta_error):
        if theta_error < 0:
            distance_error = -distance_error
        elif theta_error == 0.:
            distance_error = 0
        else:
            pass
        new_steering_angle = theta_error * 0.85 + distance_error * 0.25#some cool function for later
        return new_steering_angle
