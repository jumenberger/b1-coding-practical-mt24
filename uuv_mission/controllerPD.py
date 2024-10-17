class PDController:
    def __init__(self, Kp: float, Kd: float):
        #Initialisation the controller variables
        self.Kp=Kp
        self.Kd=Kd
        self.previous_error=0
        #to prevent error in the proprotional derivative term