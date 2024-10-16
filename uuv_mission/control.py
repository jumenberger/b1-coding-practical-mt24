class Controller:
    def __init__(self):
        self.kp = 0.15  # Gain constants
        self.kd = 0.6

        self.prev_error = 0  # error at time t=0 is zero

    def set_gains(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd
        
    def set_prev_error(self, error: float):
        self.prev_error = error
    
    def get_action(self, error: float):
        action = self.kp * error + self.kd * (error - self.prev_error)
        self.set_prev_error(error)
        return action