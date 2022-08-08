# Mobile Robotics Project
Mobile Robotics Exam Project - University of Verona - A.Y. 2021/2022

# Purpose
The goal of this project is to improve the wall following algorithm developed in the lab, implementing feature extraction approaches from range data.
The Ransac algorithm was exploited to fit lines from the turtlebot3 lidar data, the fitted lines correspond to the environmental walls.
During the execution, the robot will align itself with the fitted line and follow them.

# Documentation
Definition of the lidar regions:

We maintaned the number and nomenclature of the regions used in the lab lecture: Front, Left, Right.

We choose to change how these region are constructed:

![alt text](/images/Mobile_Robotics_regions.drawio.png)

The main reason to omit the lidar data contained in the region from 120° to 240° is to be more responsive in the case where the robot is following a wall which suddenly ends and where the robot must do a search to the right to find again the wall. By omitting the data of the specified region the robot will start turning right before. From our tests we saw that the omitted data from this region do not lead to a bad enough ransac line estimation to compromise the follow wall algorithm.

The computation of these regions was made parametrically to address real lidar sensors, since is not always the case that a lidar sensor have a 1:1 correspondence between angles and number of ranges. To clarify the previous statement, in simulation we had a 1:1 correspondance, 1 ray every 1 degree, on the tested turlebot3 the LaserScan message contained only 230 rays. 

Description of the expanded FSM:

![alt text](/images/Mobile_robotics_FSM.drawio.png)

States:
* FIND_WALL: the robot will move to find a wall to follow, to guarantee to always find a wall, the angular velocity assign in this state was set at first at -0.6 m/s and continously decreased until 0.0 m/s
* WAITING: In this state the robot will stay still for a little amount of waitingCycles, this was done to be able to word with clean lidar data
* FOLLOW_WALL: the robot will move ahed with a linear velocity of 0.1 m/s
* ALIGN_LEFT: the robot will try to fit lines from the acquired range data and align itself in the direction of the closest one. If the ransac execution will not resolve any good enough line, the number of inliers will be incrementaly reduced until at least one line is resolved

State transtion:
* E1: If front region < Th & (currentState == FIND_WALL || currentState == FOLLOW_WALL)
* E2: If waitingCycles <= 0 & currentState == WAITING
* E3: If currentState == ALIGN_LEFT & aligned
* E4: If right region > TH & currentState == FOLLOW_WALL

The aligned variable was used to indicate when the robot is aligned with the chosen line from the ransac algorithm

# Clarification: Ransac
We extracted the 2d points from the LaserScan Message of the turtlebot3. From this set we applied the standard ransac algorithm to fit lines. 
In the case the robot find itself in a corner, case detectable if both the mininum value of the right and right region is below our treshold, we discard the points of the right region from the 2d points extracted from the LaserScan message. This was done to avoid fitting the wrong line corresponding to the right wall and insted be sure to fit the line corresponding to the front wall

# Clarification: ALIGN_LEFT State
The Align Left state consists of choosing the fitted ransac line closer to the robot and rotating left until the robot is aligned with the fitted line, with a certain margin of tolerance. Once the robot is aligned, the aligned variable is set to True to allow the change to the FOLLOW_WALL state.

# Clarification: Parameters
The code necessitate a script where the parameters for the rosNode are specified. 
A guideline for these scipts is the fiel real_robot_params, used for testing the code with the real turtlebot3
*The tuning of these paramaters is essential for a good performance*

# Clarification: RViz
We exploit RViz to visualize the fitted lines.
By launching the awf.launch.py, you will be able to visualize in RViz the cloud of points of the robot's lidar sensor, the cloud of points corresponding to the fitted ransac line, and the robot position

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
