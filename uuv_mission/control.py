class PDController:
    def __init__(self, kp, kd):
        """
        Initialize the PD controller.
        :param kp: Proportional gain.
        :param kd: Derivative gain.
        """
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.prev_error = 0  # Store previous error for derivative calculation

       
    def get_control_action(self, error):
        """
        Calculate the control action using the PD formula.
        :param error: The current error (difference between reference and actual depth).
        :return: Control action to adjust the submarine's depth.
        """
        # Derivative of error
        derivative = error - self.prev_error

        # PD control law: u[t] = kp * e[t] + kd * (e[t] - e[t-1])
        control_action = self.kp * error + self.kd * derivative

        # Update previous error for next step
        self.prev_error = error

        return control_action