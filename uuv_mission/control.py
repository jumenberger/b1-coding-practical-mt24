import numpy as np

class PDController:
    """
    A simple Proportional-Derivative (PD) feedback controller.
    """
    def __init__(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0.0

    def calculate_control_action(self, error: float) -> float:
        """
        Calculates the control action based on the current error.
        The control law is: u[t] = Kp * e[t] + Kd * (e[t] - e[t-1])
        """
        # Calculate the derivative of the error
        error_derivative = error - self.previous_error

        # Calculate the control action
        control_action = (self.kp * error) + (self.kd * error_derivative)

        # Update the previous error for the next time step
        self.previous_error = error

        return control_action