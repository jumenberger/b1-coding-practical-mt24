class PDController:
    #Proportional-Derivative Controller for controlling the UUV's depth

    def __init__(self, kp=0.15, kd=0.6):
        # Store gains
        self.kp = kp
        self.kd = kd
        # Initialize previous error to 0
        self.prev_error = 0.0
    
    def __call__(self, reference: float, observation: float) -> float:
        # Calculate the error
        error = reference - observation
        
        # Calculate control action based on the reference and observation
        control_action = self.kp * error + self.kd * (error - self.prev_error)
        
        # Update previous error
        self.prev_error = error

        # Return control action
        return control_action

    def reset(self):
        #Reset the controller to its initial state
        self.prev_error = 0.0