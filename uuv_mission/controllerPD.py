class PDController:
    def __init__(self, Kp: float, Kd: float):
        #Initialisation the controller variables
        self.Kp=Kp
        self.Kd=Kd
        #to prevent error in the proprotional derivative term
        self.previous_error=0
    def controller_action(self, error: float) -> float:
        #Using the preliminary controller design equation
        difference = error - self.previous_error
        output = self.Kp * error + self.Kd * difference
        #Update the error term
        self.previous_error = error
        return output