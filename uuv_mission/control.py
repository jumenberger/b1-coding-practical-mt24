class PDController:
    """Proportional-derivative controller for the UUV depth."""

    def __init__(self, kp=0.15, kd=0.6):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0   # e[-1] = 0, as the brief specifies

    def update(self, reference, observation):
        """Return the control action for one time step."""
        error = reference - observation
        u = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        return u