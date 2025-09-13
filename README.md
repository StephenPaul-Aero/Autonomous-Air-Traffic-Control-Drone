autonomous-drone-traffic-control
Supplementary material & code for the IEEE paper “Design and Development of Autonomous Drone Traffic Control System” — AIMLA 2025.

# 🚁 Autonomous Drone Traffic Control System

**Supplementary material and code for the IEEE paper**  
**“Design and Development of Autonomous Drone Traffic Control System”** — 2025 3rd International Conference on Artificial Intelligence and Machine Learning Applications (AIMLA), IEEE. 

---

## 📌 Short abstract
This project implements an autonomous, sensor-fusion based mobile ATC drone that augments air traffic management in urban and remote areas. It integrates LiDAR, radar, ADS-B, GPS/IMU, and AI-driven routing for collision avoidance and dynamic traffic coordination.

---

## 🔑 Key results (from paper)
- **Collision avoidance success rate:** **98.7%**.  
- **Average flight path deviation (signal-weak areas):** **1.5 m**.  
- **Average response time to traffic changes:** **1.2 s**. 

---

## 🗂 Repository structure
1. src/ros_nodes/atc_demo.launch
<launch>
  <!-- ROS launch file for ATC Drone Demo -->
  <node pkg="roscore" type="roscore" name="roscore"/>
  <node pkg="gazebo_ros" type="gazebo" name="gazebo" args="-u worlds/urban_env.world"/>
  <node pkg="rospy" type="traffic_manager.py" name="traffic_manager" output="screen"/>
</launch>

2. src/sensor_fusion/sensor_fusion.py
"""
Sensor Fusion Module
Combines LiDAR, Radar, IMU, and GPS data for UAV situational awareness.
Placeholder script - extend with actual ROS subscriber/publisher logic.
"""

import numpy as np

class SensorFusion:
    def __init__(self):
        self.lidar = None
        self.radar = None
        self.gps = None
        self.imu = None

    def fuse_data(self, lidar, radar, gps, imu):
        """ Example fusion: averaging positions """
        self.lidar = lidar
        self.radar = radar
        self.gps = gps
        self.imu = imu
        fused_position = np.mean([lidar, radar, gps], axis=0)
        return fused_position

if __name__ == "__main__":
    sf = SensorFusion()
    pos = sf.fuse_data([1,2,3],[1.1,2.1,3.1],[0.9,2,3.2],[0,0,0])
    print("Fused Position:", pos)

3. src/control/a_star.py
"""
A* Path Planning Algorithm
Used for UAV route optimization and collision avoidance.
"""

import heapq

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal, neighbors):
    queue = [(0, start)]
    came_from = {}
    cost = {start: 0}

    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            break
        for next_node in neighbors(current):
            new_cost = cost[current] + 1
            if next_node not in cost or new_cost < cost[next_node]:
                cost[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                heapq.heappush(queue, (priority, next_node))
                came_from[next_node] = current
    return came_from

if __name__ == "__main__":
    def neighbors(n):
        x, y = n
        return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
    path = astar((0,0), (3,3), neighbors)
    print("Computed path:", path)

4. src/control/rl_navigation.py
"""
Reinforcement Learning Navigation (Placeholder)
Framework for UAV path optimization using Q-learning.
"""

import numpy as np

class RLNavigation:
    def __init__(self, n_states=10, n_actions=4, alpha=0.1, gamma=0.9):
        self.q_table = np.zeros((n_states, n_actions))
        self.alpha = alpha
        self.gamma = gamma

    def update(self, state, action, reward, next_state):
        best_next = np.max(self.q_table[next_state])
        self.q_table[state, action] += self.alpha * (reward + self.gamma * best_next - self.q_table[state, action])

if __name__ == "__main__":
    rl = RLNavigation()
    rl.update(0, 1, 10, 2)
    print("Q-Table after update:\n", rl.q_table)

5. notebooks/01-data-analysis.ipynb (Jupyter Notebook – paste as JSON)

Create a new notebook in GitHub and paste the JSON below:

{
 "cells": [
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "# Data Analysis for Autonomous Drone ATC\n",
    "This notebook contains simulation data preprocessing, path deviation plots, and collision risk analysis."
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 1,
   "metadata": {},
   "outputs": [],
   "source": [
    "import matplotlib.pyplot as plt\n",
    "import numpy as np\n",
    "\n",
    "# Example data (replace with real logs later)\n",
    "traffic_density = [1, 2, 3, 4, 5]\n",
    "collision_risk = [0, 0.2, 0.5, 1.0, 1.2]\n",
    "\n",
    "plt.plot(traffic_density, collision_risk, marker='o')\n",
    "plt.xlabel('Traffic Density (relative scale)')\n",
    "plt.ylabel('Collision Risk (%)')\n",
    "plt.title('Collision Risk vs Traffic Density')\n",
    "plt.grid(True)\n",
    "plt.show()"
   ]
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.9"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}

6. simulations/matlab/trajectory_optimization.m
% Trajectory Optimization Script (MATLAB)
% Placeholder for UAV trajectory optimization

disp('Running trajectory optimization...');

time = linspace(0,10,100);
x = sin(time);
y = cos(time);

plot(x,y,'b-','LineWidth',2);
xlabel('X Position');
ylabel('Y Position');
title('Optimized UAV Trajectory (Placeholder)');
grid on;

7. results/figures/README.md
# Figures & Results

This folder contains:
- Collision risk vs traffic density plots
- Flight path deviation plots
- Battery endurance analysis
- Simulation screenshots (Gazebo, MATLAB)
