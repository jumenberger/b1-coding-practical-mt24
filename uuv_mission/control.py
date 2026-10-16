# Implementation of PD controller
class Controller:
    def __init__(self, Kp: float = 0.15, Kd: float = 0.6, init_error: float = 0.0):

        self.Kp = Kp
        self.Kd = Kd
        self.error = init_error

    def __call__(self, observation: float, reference: float) -> float:
        error = reference - observation
        derivative = error - self.error
        self.error = error
        return self.Kp * error + self.Kd * derivative

