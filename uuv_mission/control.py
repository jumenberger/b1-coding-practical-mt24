class PDController:
    def __init__(self, kp: float = 0.15, kd: float = 0.6):
        self.kp = kp
        self.kd = kd
        self.previous_error = None

    def reset(self):
        self.previous_error = None

    def __call__(self, reference: float, output: float) -> float:
        if self.previous_error is None:
            self.previous_error = 0

    
        error = reference - output
        action = self.kp * error + self.kd * (error - self.previous_error)
        self.previous_error = error
        return action