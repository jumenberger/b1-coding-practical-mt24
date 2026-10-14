class PDController:

    def __init__(self, kp, kd):
        self.kp = kp # Proportional gain
        self.kd = kd # Derivative gain
        self.prev_error = 0.0 # Assumption from specification

    def compute_control(self, reference, observation):
        error = reference - observation
        error_change = error - self.prev_error 

        # Implementing the formula u(t) = Kp * e(t) + Kd * (e(t) - e(t-1)) from the specification
        control_signal = self.kp * error + self.kd * error_change 

        # Update the previous error for the next iteration
        self.prev_error = error

        return control_signal