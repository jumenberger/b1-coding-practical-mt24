from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from terrain import generate_reference_and_limits
import csv
from control import PD_controller

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

    #Initialising the arrays
    @classmethod
    def __init__(self, reference, cave_height, cave_depth):
        self.reference = reference
        self.cave_height = cave_height
        self.cave_depth = cave_depth


    @classmethod
    def random_mission(cls, duration: int, scale: float):
        (reference, cave_height, cave_depth) = generate_reference_and_limits(duration, scale)
        return cls(reference, cave_height, cave_depth)

    @classmethod
    def from_csv(cls, file_name: str):
        with open("data/%s.csv" % (file_name)) as file:
            for row in file:
                ref = row.split(',')[0]
                if ref != 'reference':
                    Mission.reference.append(float(ref))

                height = row.split(',')[1]
                if height != 'cave_height':
                    Mission.cave_height.append(float(height))

                #Each row in the csv actually ends with /n, which we need to remove
                full_value = row.split(',')[2]
                shortened_value = full_value[:-1]
                if shortened_value != 'cave_depth':
                    Mission.cave_depth.append(float(shortened_value))
        
       
        return Mission



class ClosedLoop:
    #def __init__(self, plant: Submarine, controller):
    def __init__(self, plant: Submarine):
        self.plant = plant
        #self.controller = controller

    def simulate(self,  mission: Mission, disturbances: np.ndarray) -> Trajectory:

        T = len(mission.reference)
        if len(disturbances) < T:
            raise ValueError("Disturbances must be at least as long as mission duration")
        
        positions = np.zeros((T, 2))
        actions = np.zeros(T)
        self.plant.reset_state()

        #initialising the initial error
        e_0 = float(mission.reference[0])-float(self.plant.get_depth())

        for t in range(T):
            positions[t] = self.plant.get_position()
            observation_t = self.plant.get_depth()
            Kp = 0.125
            Kd = 0.65
            [e_0, u_t] = PD_controller(mission.reference[t],observation_t, e_0, Kp, Kd)
            actions[t] = u_t

            self.plant.transition(actions[t], disturbances[t])

        return Trajectory(positions)
        
    def simulate_with_random_disturbances(self, mission: Mission, variance: float = 0.5) -> Trajectory:
        disturbances = np.random.normal(0, variance, len(mission.reference))
        return self.simulate(mission, disturbances)

#Initialse and create an instance of the Mission class
Mission([],[],[])
Mission.from_csv("mission")

#Initialising the submarine and modeling it's course
sub = Submarine()
closed_loop = ClosedLoop(sub)
trajectory = closed_loop.simulate_with_random_disturbances(Mission)
trajectory.plot_completed_mission(Mission)