class PDController:
    """Proportional-Derivative Controller for controlling the UUV's depth."""

    def __init__(self, kp=0.15, kd=0.6):
        # store gains
        self.kp = kp
        self.kd = kd
        # initialize previous error to 0
        self.prev_error = 0.0
    
    def control(self, reference, observation):
        """Calculate the control action based on the reference and observation."""
        # calculate error
        error = reference - observation
        
        # calculate control action
        control_action = self.kp * error + self.kd * (error - self.prev_error)
        
        # update previous error
        self.prev_error = error

        # return control action
        return control_action