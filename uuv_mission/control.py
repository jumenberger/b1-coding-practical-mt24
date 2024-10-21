class Controller:
    def __init__(self):
        self.Kp = 0.15
        self.Kd = 0.6
        self.prev_error = 0

    def get_action(self, reference_t, observation_t):
        # You are required to implement this method
        error = reference_t - observation_t
        error_dot = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Kd * error_dot