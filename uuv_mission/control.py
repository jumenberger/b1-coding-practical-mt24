class PIDController:
    # Constructor to initialize PID controller
    def __init__(self, KP, KI, KD, integral_limit):
        self.KP = KP
        self.KD = KD
        self.KI = KI
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_limit = integral_limit

    def reset(self):
        # Reset integral and previous error for multiple test runs
        self.prev_error = 0.0
        self.integral = 0.0

    def compute_control(self, reference, measurement):

        error = reference - measurement
        self.integral += error

        # Apply anti-windup by clamping the integral within specified limits
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        derivative = error - self.prev_error

        # Calculate the control signal using PID formula.
        control = self.KP * error + self.KI * self.integral + self.KD * derivative
        
        # Store error for next derivative calculation
        self.prev_error = error

        # Return computed control signal
        return control

    def __call__(self, reference, measurement):
        # Make the object callable: allows direct use as controller(reference, measurement). This is utilised when completing the ClosedLoop class.
        return self.compute_control(reference, measurement)