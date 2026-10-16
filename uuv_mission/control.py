# Implementation of PD controller
class Controller:
    def __init__(self, Kp: float = 0.15, Kd: float = 0.0):
        self.Kp = Kp
        self.Kd = Kd