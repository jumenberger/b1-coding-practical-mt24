# A PD controller in form u(t) = Kp*e(t) + Kd*(e(t) - e(t-1))
class Controller:
    def __init__(self):
        self.happy = True

    # Define how the control action is computed
        
    def compute_control(self, reference, measurement, prev_reference, prev_measurement):
        self.Kp = 0.15
        self.Kd = 0.6
        error = reference - measurement
        prev_error = prev_reference - prev_measurement
        derivative = error - prev_error
        u = self.Kp * error + self.Kd * derivative
        return u