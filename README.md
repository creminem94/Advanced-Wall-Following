# Mobile Robotics Project
Mobile Robotics Exam Project - University of Verona - A.Y. 2021/2022

# Purpose
The goal of this project is to improve the wall following algorithm developed in the lab, implementing feature extraction approaches from range data.
The Ransac algorithm was exploited to fit lines from the turtlebot3 lidar data, the fitted lines correspond to the environmental walls.
During the execution, the robot will align itself with the fitted line and follow them.

# Documentation
Definition of the lidar regions:
We maintaned the number and nomenclature of the regions used in the lab lecture: Front, Left, Right
We choose to change how these region are constructed:
![alt text](/images/Mobile_Robotics_regions.drawio.png)

Description of the expanded FSM:

![alt text](/images/Mobile_robotics_FSM.drawio.png)

States:
1) Find wall: the robot will move to find a wall to follow, to guarantee to always find a wall, the angular velocity assign in this state was set at first at -0.6 m/s and continously decreased until 0.0 m/s
2) Waiting: In this state the robot will stay still for a little amount of waitingCycles, this was done to be able to word with clean lidar data
3) Follow Wall: the robot will move ahed with a linear velocity of 0.1 m/s
4) Align Left: the robot will try to fit lines from the acquired range data and align itself in the direction of the closest one. If the ransac execution will not resolve any good enough line, the number of inliers will be incrementaly reduced until at least one line is resolved

State transtion:
E1) If front region < Th & (currentState == FIND_WALL || currentState == FOLLOW_WALL)
E2) If waitingCycles <= 0 & currentState == WAITING
E3) If currentState == ALIGN_LEFT & aligned
E4) If right region > TH & currentState == FOLLOW_WALL

# Steps for running the code
Simulation
open Turtlebot3UnityROS2 project via Unity Hub
visualizer - ros2 launch advanced_wall_following awf.launch.py 
ros2 node  - ros2 run advanced_wall_following advanced_wall_following 

Turtlebot3
ssh ubuntu@<robot_ip> - pw: turtlebot
ros2 launch turtlebot3_bringup robot.launch.py
visualizer - ros2 launch advanced_wall_following awf.launch.py 
ros2 node  - ros2 run advanced_wall_following advanced_wall_following 
