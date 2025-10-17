import numpy as np

class PDController:
    
    def __init__(self, Kp: float = 0.15, Kd: float = 0.6): # Provided values
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0.0 # Provided value
        self.is_first_step = True
    
    def compute_control_action(self, current_error: float) -> float:
        proportional = self.Kp * current_error
        
        if self.is_first_step:
            derivative = 0.0
            self.is_first_step = False
        else:
            derivative = self.Kd * (current_error - self.previous_error)
        
        control_action = proportional + derivative
        self.previous_error = current_error
        return control_action
    
    def reset(self):
        self.previous_error = 0.0
        self.is_first_step = True
