class Controller:
    # define the KD controller parameters
    def __init__(self):
        self.Kp = 0.15
        self.Kd = 0.6

    # define the KD controller
    def KD_Controller(self, reference: float, observation: float, lasterror: float):
        error = reference - observation
        derivative = error - lasterror
        action = self.Kp*error + self.Kd*derivative
        return action
