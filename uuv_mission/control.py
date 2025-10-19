# a proportional derivative (PD) feedback controller

def pd_controller(error, prev_error, Kp=0.15, Kd=0.6) -> float:
    
    control_action = Kp * error + Kd * (error - prev_error)
    return control_action