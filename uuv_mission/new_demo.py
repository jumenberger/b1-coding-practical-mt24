
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from dynamic import Submarine   # Import Submarine class
from dynamic import ClosedLoop  # Import ClosedLoop class
from dynamic import Mission  # Import Mission class
from new_controller import new_controller1
# Instantiate the Submarine
sub = Submarine()

# Instantiate your controller with gains


# Create the closed-loop system with the submarine and controller
closed_loop = ClosedLoop(sub, new_controller1)

# Load the mission from a CSV file (ensure the path is correct)
mission = Mission.from_csv("C:/Users/Josh/B1 eng comp/b1-coding-practical-mt24/uuv_mission/mission.csv")

print("Mission object:", mission)
print("Mission reference:", mission.reference)

# Simulate the closed-loop system with random disturbances
trajectory = closed_loop.simulate_with_random_disturbances(mission)

# Plot the completed mission trajectory
trajectory.plot_completed_mission(mission)
