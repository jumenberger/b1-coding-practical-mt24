class Controller:
    def __init__(self, Kp=0.15, Kd=0.6, Ki=0):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0

        # for integration term
        self.Ki = Ki
        self.error_sum = 0

    def get_action(self, reference_t, observation_t):
        # You are required to implement this method
        error = reference_t - observation_t
        error_dot = error - self.prev_error
        self.prev_error = error
        self.error_sum += error
        return self.Kp * error + self.Kd * error_dot + self.Ki * self.error_sum