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
    Create3/
    ├── build/                      # Added to .gitignore, so not tracked by GitHub.
    ├── install/                    # Added to .gitignore, so not tracked by GitHub.
    ├── log/                        # Added to .gitignore, so not tracked by GitHub.
    ├── src/                        # The source folder containing the packages.
    │   ├── create3_sim/            # The package for the Create-3 simulation.
    │   └── create3_control/        # The package for controlling the Create-3.
    │       ├── config/             # Contains the YAML file specifying the target end position for the robot.
    │       ├── create3_control/    # The folder storing the python script for moving the iRobot. 
    │       ├── launch/             # The folder containing the launch files for launching the node for moving the robot. 
    │       ├── package.xml
    │       ├── setup.cfg
    │       └── setup.py
    ├── video/                      # The folder supposed to contain the video.
    ├── .gitignore
    ├── README.md
    └── test.py                     # Simple Python test file.

## Running the Code
1. **Launch the simulation in Gazebo** using:
    ```bash
    ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
    ```
    
2. In a different terminal,**use the already provided service for undocking** the iRobot Create-3:
    ```bash
    ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
    ```
    The robot will move away from the docking station and turn 180°.
   
3. In general you can **move the robot** by publishing geometry_msgs/msg/Twist messages to the /cmd_vel topic (in a different terminal) as follows:
    ```bash
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```
    However to automate the robot to go towards a chosen target position and using something else than a straight line or circular arch to reach this target position, a node was developed in the create3_control       package. Running the node can be achieved with the command:
    ```bash
    ros2 launch create3_control move_robot.launch.py
    ```
    The purpose of this package is to move the robot according to a certain trajectory to the target position, calculate the commands to publish to the /cmd_vel topic as Twist messages to achieve this. Afterwards     the robot moves back to its start position in order to dock. However debugging of this node and finishing its development was not possible due to hardware difficulties which did not allow the simulation to        run properly.
   
4. **Docking of the robot** is achieved with the following command:
    ```bash
    ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
    ```
    This command will only be accepted when the docking station is visible for the robot. Therefore the robot has to oriented correctly to face the docking station using the previous node to enable docking.

## Results
A video recording of the Create-3 robot undocking, following a creative path to the target, and docking back at the station. To watch the video, click on the following link and then on 'download raw file' in the top right corner. (The video is 6.1 MB, which is too big to display directly.)

[Watch the video](video/iRobotCreate3_UndockMoveDock.mp4)


