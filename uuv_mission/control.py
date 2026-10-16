class PDController:
    """Discrete-time PD controller: u[t] = KP*e[t] + KD*(e[t] - e[t-1])."""

    def __init__(self, kp: float = 0.15, kd: float = 0.6):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0  # e[-1] = 0

    def reset(self) -> None:
        """Clear controller memory so each simulation starts from e[-1] = 0."""
        self.prev_error = 0.0

    def __call__(self, reference: float, observation: float) -> float:
        error = reference - observation
        action = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        return action