# A PID controller in form u(t) = Kp*e(t) + Kd*(e(t) - e(t-1)) + Ki*sum(e(t))
class Controller:
    def __init__(self):
        self.happy = True   # Need something to initialise
        self.error_sum = 0

    # Define how the control action is computed
        
    def compute_control(self, reference, measurement, prev_reference, prev_measurement):
        self.Kp = 0.01
        self.Kd = 0.85
        self.Ki = 0.015
        error = reference - measurement
        prev_error = prev_reference - prev_measurement
        derivative = error - prev_error
        self.error_sum = error + self.error_sum
        u = self.Kp * error + self.Kd * derivative + self.Ki * self.error_sum
        return u