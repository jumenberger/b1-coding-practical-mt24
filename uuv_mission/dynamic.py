from __future__ import annotations
from dataclasses import dataclass
from new_controller import new_controller1
import numpy as np
import matplotlib.pyplot as plt
from terrain import generate_reference_and_limits
import pandas as pd

class Submarine:
    def __init__(self):

        self.mass = 1
        self.drag = 0.1
        self.actuator_gain = 1

        self.dt = 1 # Time step for discrete time simulation

        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1 # Constant velocity in x direction
        self.vel_y = 0


    def transition(self, action: float, disturbance: float):
        self.pos_x += self.vel_x * self.dt
        self.pos_y += self.vel_y * self.dt

        force_y = -self.drag * self.vel_y + self.actuator_gain * (action + disturbance)
        acc_y = force_y / self.mass
        self.vel_y += acc_y * self.dt

    def get_depth(self) -> float:
        return self.pos_y
    
    def get_position(self) -> tuple:
        return self.pos_x, self.pos_y
    
    def reset_state(self):
        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1
        self.vel_y = 0
    
class Trajectory:
    def __init__(self, position: np.ndarray):
        self.position = position  
        
    def plot(self):
        plt.plot(self.position[:, 0], self.position[:, 1])
        plt.show()

    def plot_completed_mission(self, mission: Mission):
        x_values = np.arange(len(mission.reference))
        min_depth = np.min(mission.cave_depth)
        max_height = np.max(mission.cave_height)

        plt.fill_between(x_values, mission.cave_height, mission.cave_depth, color='blue', alpha=0.3)
        plt.fill_between(x_values, mission.cave_depth, min_depth*np.ones(len(x_values)), 
                         color='saddlebrown', alpha=0.3)
        plt.fill_between(x_values, max_height*np.ones(len(x_values)), mission.cave_height, 
                         color='saddlebrown', alpha=0.3)
        plt.plot(self.position[:, 0], self.position[:, 1], label='Trajectory')
        plt.plot(mission.reference, 'r', linestyle='--', label='Reference')
        plt.legend(loc='upper right')
        plt.show()

@dataclass
class Mission:
    reference: np.ndarray
    cave_height: np.ndarray
    cave_depth: np.ndarray

    def __init__(self, reference: np.ndarray, cave_height: np.ndarray, cave_depth: np.ndarray):
        self.reference = reference
        self.cave_height = cave_height
        self.cave_depth = cave_depth

    @classmethod
    def random_mission(cls, duration: int, scale: float):
        # Assuming this method generates random mission data
        (reference, cave_height, cave_depth) = generate_reference_and_limits(duration, scale)
        return cls(reference, cave_height, cave_depth)

    @classmethod
    def from_csv(cls, file_name: str):
        # Load CSV file
        mission_data = pd.read_csv(file_name)

        # Check if required columns are present in the CSV
        required_columns = ['reference', 'cave_height', 'cave_depth']
        if not all(column in mission_data.columns for column in required_columns):
            raise ValueError(f"CSV file must contain columns: {', '.join(required_columns)}")

        # Extract the data from the CSV columns
        reference = mission_data['reference'].to_numpy()
        cave_height = mission_data['cave_height'].to_numpy()
        cave_depth = mission_data['cave_depth'].to_numpy()

        # Return an instance of Mission
        return cls(reference, cave_height, cave_depth)

class ClosedLoop:
    def __init__(self, plant: Submarine, controller):
        self.plant = plant
        self.controller = controller

    def simulate(self, mission: Mission, disturbances: np.ndarray) -> Trajectory:
        T = len(mission.reference)
        if len(disturbances) < T:
            raise ValueError("Disturbances must be at least as long as mission duration")
        
        positions = np.zeros((T, 2))
        actions = np.zeros(T)
        self.plant.reset_state()

        for t in range(T):
            # Log current position
            positions[t] = self.plant.get_position()

            # Get the current depth (observation) and the reference depth
            observation_t = self.plant.get_depth()
            reference_t = mission.reference[t]
            previous_pos = positions[t-1] if t > 0 else positions[t]
            prev_reference = mission.reference[t-1] if t > 0 else mission.reference[t]

            # Compute control action
            actions[t] = new_controller1(reference_t, observation_t, previous_pos, prev_reference)

            # Apply control action and disturbance
            self.plant.transition(actions[t], disturbances[t])

        return Trajectory(positions)

    def simulate_with_random_disturbances(self, mission: Mission, variance: float = 0) -> Trajectory:
        # Generate random disturbances based on a normal distribution
        disturbances = np.random.normal(0, variance, len(mission.reference))
        # Call the simulate method with the generated disturbances
        return self.simulate(mission, disturbances)
