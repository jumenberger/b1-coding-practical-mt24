class PDController:
    """Proportional-derivative controller for submarine depth."""

    KP = 0.15
    KD = 0.6

    def __init__(self):
        # The first derivative term compares the first error with zero.
        self.previous_error = 0.0

    def compute_control(self, target_depth: float, actual_depth: float) -> float:
        """Return the control action for the current target and actual depths."""
        error = target_depth - actual_depth
        action = self.KP * error + self.KD * (error - self.previous_error)
        self.previous_error = error
        return action
