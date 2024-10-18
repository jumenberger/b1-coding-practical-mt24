class PDController:
    def __init__(self, Kp=0.15, Kd=0.6):
        self.Kp = Kp  # Proportional gain
        self.Kd = Kd  # Derivative gain
        self.previous_error = 0  # Initialize previous error to 0

    def compute_control_action(self, error):
        # Compute the control output u[t] using PD control law
        control_action = self.Kp * error + self.Kd * (error - self.previous_error)
        # Update previous error
        self.previous_error = error
        return control_action