class PD_Controller:
    def __init__(self, Kp, Kd):
        self.proportional_gain = Kp
        self.derivative_gain = Kd

    def get_action(self, error, error_dot):
        return self.proportional_gain * error + self.derivative_gain * error_dot
