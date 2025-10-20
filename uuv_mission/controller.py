# A PD controller in form u(t) = Kp*e(t) + Kd*(e(t) - e(t-1))
class Controller:
    def __init__(self, Kp: float, Kd: float):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0.0   # Initialize previous error to zero
    
    def compute_control(self, reference: float, measurement: float) -> float:
        error = reference - measurement
        derivative = error - self.prev_error
        u = self.Kp * error + self.Kd * derivative
        self.prev_error = error
        return u