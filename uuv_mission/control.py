from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from .terrain import generate_reference_and_limits

# Defining the constants
K_d =  0.6
K_p = 0.15

# Define control function
def control_submarine(y_0, r_0, y_1, r_1):
    e_0 = r_0 - y_0
    e_1 = r_1 - y_1
    u_1 = K_p * e_1 + K_d * (e_1 - e_0)
    return u_1

