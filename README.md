# iRobot Create-3 Path Planning Project
This project involves path planning for the iRobot Create-3 robot in a simulated environment.

## Overview
 The goal is to program the robot to:

1. **Undock** from its docking station using the provided undocking service.
2. **Move from Point A to Point B**: 
   - **Start Position**: The robot's initial position after undocking.
   - **End Position**: A target point read from a YAML configuration file.
   - The robot's trajectory should be creative and **not follow a straight line or circular arc**.
3. **[Optional] Dock**: After reaching the target point, the robot may redock at its original docking station.

## Requirements
- **Operating System**: Ubuntu 22 with ROS2 Humble installed.
- **Simulator**: Create-3 Gazebo simulator.

## Setup Instructions
1. Clone and build the Create-3 simulator repository from iRobot.
2. Launch the Create-3 simulation in Gazebo using:
   ```bash
   ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
   ```
   
## Project Structure
- **src/**: Contains the ROS2 package with the code for undocking, path planning, and docking.
- **config/**: Contains the YAML file specifying the target end position for the robot.
- **video/**: A screen recording of the robot performing the task in simulation.
- **README.md**: This overview and instructions for setting up and running the project.

## Running the Code
1. Launch the simulation in Gazebo.
2. Run the main script to start the undocking, path planning, and optional docking actions.

## Results
- A video recording of the Create-3 robot undocking, following a creative path to the target, and optionally docking back at the station.

## Extra Notes
- The code and video are organized for ease of understanding, with comments in the code explaining each step of the process.
