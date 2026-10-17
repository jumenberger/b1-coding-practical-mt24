class PDcontroller:
    def __init__(self):
        self.previous_error = 0
    def get_action(self, reference, observation):
        error = reference - observation
        action = 0.15*error + 0.6*(error-self.previous_error)
        self.previous_error=error
        return action
