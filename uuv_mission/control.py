class PDController:
    def __init__(self):
        self.K_p = 0.15  # Proportional gain
        self.K_d = 0.6  # Derivative gain
        self.reset()  # Initialize previous error to 0

    def reset(self):
        # Function to initialise the initial previous error to 0
        self.previous_error = 0
   
    def compute(self, reference, observation):
        # Calculate error term for this time step
        error = reference - observation

        # Compute control signal using current and previous error terms
        u = self.K_p * error + self.K_d * (error - self.previous_error)

        # Update previous error for next time step
        self.previous_error = error
        return u