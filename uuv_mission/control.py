import numpy as np

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
        return float(reference - depth)


    def get_action(self, reference: float, depth: float, last_error: float):
        """get the current controller action (force in y-direction).
        
        parameters:
        """
        error = self.get_error(reference, depth)
        action = self._kp * error + self._kd * (error - last_error)
        return action