class PDController:
    def __init__(self):
        self.kp = 0.15
        self.kd = 0.6
        self.previous_error = 0.0

    def compute(self, reference, observation):
        error = reference - observation

        action = self.kp * error + self.kd * (error - self.previous_error)

        self.previous_error = error

        return action