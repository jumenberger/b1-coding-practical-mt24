class Controller:
    def __init__(self):
        self.KP = 0.15
        self.KD = 0.6
        self.previous_error = 0

    def control(self, reference, observation):
        error = reference - observation

        action = (
            self.KP * error
            + self.KD * (error - self.previous_error)
        )

        self.previous_error = error

        return action
