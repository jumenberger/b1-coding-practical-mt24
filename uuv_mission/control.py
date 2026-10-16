
class PDController:
    def __init__(self, kp=0.15, kd=0.6):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0.0

    def __call__(self, reference, observation):
        error = reference - observation

        action = (
            self.kp * error
            + self.kd * (error - self.previous_error)
        )

        self.previous_error = error

        return action
