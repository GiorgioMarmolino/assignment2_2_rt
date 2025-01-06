### RESEARCH TRACK - ASSIGNMENT 2, PART 2 - Implementation of a ROS2 node

Made by: Marmolino Giorgio
a.y. 2024/2025

## Project introduction
In this project a ROS2 node has been implemented in C++ to simulate a simple logic of motion for a mobile robot in its simulation environment (without edges, wall or obstacles). This project has been developed starting from the original package of prof. Recchiuto:

https://github.com/CarmineD8/robot_urdf.git

## Node descriptions
The node is implemented in order to control the robot following in loop a pre-designed pattern: it moves straight until reaching a coordinate-threshold, so it turns and go on moving straight in the opposite direction; inside the C++ node, the class ROS2Robot has been implemented: here we can find the 'odom' topic subscriber and a publisher to the topic 'cmd_vel' (where it's possible for the control node to publish the velocity values for the mobile robot). The implemented logic ensures that the robot adjusts its movement based on its x-coordinate. If the x-coordinate falls below 1 or exceeds 9, an angular velocity, either -1 or 1 depending on the required turning direction, is applied to steer the robot and prevent it from moving out of bounds. However, if the x-coordinate remains within the specified range, the robot proceeds in a straight path, driven solely by a linear velocity without any angular adjustments. The node code is minimal since it's focused on initializing the ROS2 environment: the procedure consists of creating a node with the `ROS2Robot` class, launching it, and keeping it active using `spin()` until the program concludes. Afterward, the ROS2 runtime is gracefully terminated, and resources are freed using `shutdown()`. This ensures a streamlined and maintainable implementation while achieving the intended motion behavior for the mobile robot.

## Prerequisites
Before executing the project you are required to: 
1) Install Gazebo, the 3D simulator for the ROS;
2) Install Rviz, a tool for ROS visualization that allows the user to view the simulated mobile robot model with all its sensors and related informations; it is usefull for debug purposes;
3) Xterm;
4) python3, since part of the code is written in python3;
5) C++ and g++ compiler;
6) Robot Operating System (known as ROS2);

## Running the project
After cloning this repository into your src/ folder of your ROS2 workspace:
```bash
git clone https://github.com/GiorgioMarmolino/assignment2_1_rt
```
go back in the main folder of your ROS2-WS and build this package:
```bash
colcon build
```
```bash
source install/local_setup.bash
```
then launch the project using the launch file:
```bash
ros2 launch src/assignment2_2_rt/launch/gazebo.launch.py
```

Now the whole software has been launched, so we can see:
1) The control node of the robot that implements the control logic;
2) The simulation environment

