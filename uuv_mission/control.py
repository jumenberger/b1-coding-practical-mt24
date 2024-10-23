class PDController:
    def __init__(self, Kp, Kd):      #initialise PD controller with gains and error
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0

    def compute(self, error):           #calculate control action based on current error
        derivative_error = error - self.previous_error
        control_action = self.Kp * error + self.Kd * derivative_error
        self.previous_error = error
        return control_action
