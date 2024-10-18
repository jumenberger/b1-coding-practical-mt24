"""Python module for controller"""

class Controller:
    """Class for controller of the submarine in the cave.
    
    The purpose is to take the reference depth and the actual submarine depth as inputs, 
    and output the action (force by the controller in the y-direction)
    """
    def __init__(self, kp: float = 0.15, kd: float = 0.6, ki: float = 0.0):
        # controller constants (given in the pdf)
        self._kp = kp
        self._kd = kd
        self._ki = ki  # not used in this assignment


    def get_error(self, reference: float, depth: float) -> float:
        """Get the error between reference signal and output signal."""
        return float(reference - depth)


    def get_action(self, reference: float, depth: float, last_error: float):
        """Get the current controller action (force in y-direction).
        
        Parameters:
            reference (float): reference depth given by the csv file
            depth (float): current y position of submarine
            last_error (float): error from last time step
        """
        error = self.get_error(reference, depth)
        action = self._kp * error + self._kd * (error - last_error)
        return action
