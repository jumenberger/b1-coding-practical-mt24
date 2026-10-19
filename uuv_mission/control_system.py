"""Proportional-derivative control for the UUV mission.

After initialising the object with the required constants the action_calculator method calculates the control action from the target and actual positions.
The constants relating to proportional and derivative control may be set when initialising the class.

Typical usage example:
    Submarine_PDController = PD_Controller(0.15,0.6)
    action = Submarine_PDController.action_calculator(t,target_position,actual_position)
"""

import numpy as np

class PD_Controller:
    def __init__(self, K_P: float, K_D: float):
        self.K_P = K_P
        self.K_D = K_D

    def action_calculator(self,t,target_position: np.ndarray,actual_position: np.ndarray):
        
        error_position = target_position - actual_position[:, 1]
        action = self.K_P*(error_position[t]) + self.K_D*(error_position[t]-error_position[t-1])

        return action