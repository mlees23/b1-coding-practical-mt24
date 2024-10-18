from __future__ import annotations
from dataclasses import dataclass
import sys
import os
# Add the parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
print(sys.path) 
import numpy as np
import matplotlib.pyplot as plt
import csv
from uuv_mission.terrain import generate_reference_and_limits
from uuv_mission.control import PDController 

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
#class Mission:
    #reference: np.ndarray
    #cave_height: np.ndarray
    #cave_depth: np.ndarray

class Mission:
    def __init__(self, reference: float, cave_height: float, cave_depth: float):
        self.reference = reference  # Desired height
        self.cave_height = cave_height  
        self.cave_depth = cave_depth  
        #self.controller = PDController(kp=0.15, kd=0.6)  # Initialize PD controller


    @classmethod
    def random_mission(cls, duration: int, scale: float):
        (reference, cave_height, cave_depth) = generate_reference_and_limits(duration, scale)
        return cls(reference, cave_height, cave_depth)


    @classmethod
    def from_csv(cls, file_path: str) -> 'Mission':
         # Initialize empty lists to store values from CSV
        references = []
        cave_heights = []
        cave_depths = []

        # Open the CSV file and read its contents
        with open(file_path, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                # Append the values to the lists
                references.append(float(row['reference']))  # Convert to float and append to list
                cave_heights.append(float(row['cave_height']))  # Convert to float and append
                cave_depths.append(float(row['cave_depth']))  # Convert to float and append
                # Return an instance of the class with lists as attributes
        return cls(references, cave_heights, cave_depths)

   

class ClosedLoop:
    def __init__(self, plant: Submarine, controller: PDController):
        self.plant = plant
        self.controller = controller

    def simulate(self,  mission: Mission, disturbances: np.ndarray) -> Trajectory:

        T = len(mission.reference)
        if len(disturbances) < T:
            raise ValueError("Disturbances must be at least as long as mission duration")
        
        positions = np.zeros((T, 2))
        actions = np.zeros(T)
        self.plant.reset_state()
        

        for t in range(T):
            positions[t] = self.plant.get_position()
            # Call your controller here
            current_depth = self.plant.get_depth()
            current_reference = mission.reference[t]
            actions[t] = self.controller.compute_control(current_depth,current_reference)
            self.plant.transition(actions[t], disturbances[t])

        return Trajectory(positions)
        
    def simulate_with_random_disturbances(self, mission: Mission, variance: float = 0.5) -> Trajectory:
        disturbances = np.random.normal(0, variance, len(mission.reference))
        return self.simulate(mission, disturbances)
    

#sub = Submarine()
# Instantiate your controller (depending on your implementation)
#controller = PDController(kp=0.06, kd=0.76)
#closed_loop = ClosedLoop(sub, controller)
#mission = Mission.from_csv(os.path.join('data', 'mission.csv'))  # Adjust path to the CSV file


#trajectory = closed_loop.simulate_with_random_disturbances(mission)
#trajectory.plot_completed_mission(mission)
