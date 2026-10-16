class PDController:
    """Discrete-time proportional-derivative controller.

    u[t] = KP * e[t] + KD * (e[t] - e[t-1]),  with e[t] = r[t] - y[t] and e[-1] = 0.
    """

    def __init__(self, kp: float = 0.15, kd: float = 0.6):
        self.kp = kp
        self.kd = kd
        self.reset()

    def reset(self) -> None:
        """Clear the controller's memory so that e[-1] = 0."""
        self.prev_error = 0.0

    def compute_action(self, reference: float, measurement: float) -> float:
        error = reference - measurement
        action = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        return action

    def __call__(self, reference: float, measurement: float) -> float:
        return self.compute_action(reference, measurement)
