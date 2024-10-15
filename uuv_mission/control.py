import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def PD_controller(reference_t1,depth_t1,e_t0,Kp,Kd):
    # Calculates the control action at time t1 based on the current state and past state at time t0
    
    e_t1 = reference_t1-depth_t1
    u_t = Kp * e_t1 + Kd * (e_t1 - e_t0)
    

    return e_t1, u_t