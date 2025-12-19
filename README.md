# EBU6350 Group 4: Autonomous Maze Navigation

This repository contains the source code for the "Autonomous Maze Navigation using MicroROS-ESP32" project.

## Project Structure

* **firmware**: C/C++ code for the ESP32 microcontroller using the MicroROS Arduino framework.
* **simulation**: Python-based ROS2 package for Gazebo simulation.
* **docs**: Project documentation.

## Key Features

1.  **MicroROS Integration**: Seamless communication between ESP32 and ROS2 Humble.
2.  **Autonomous Navigation**: LIDAR-based obstacle avoidance and PID heading control.

## How to Run Simulation

cd simulation
colcon build
source install/setup.bash
ros2 launch ebu6350_simulation simulation.launch.py
