import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def PD_controller(reference_t1,depth_t1,e_t0,Kp,Kd):
    # Calculates the control action at time t1 based on the current state and past state at time t0
    
    e_t1 = float(reference_t1)-float(depth_t1)
    u_t = float(Kp) * float(e_t1) + float(Kd) * (float(e_t1) - float(e_t0))
    

    return e_t1, u_t