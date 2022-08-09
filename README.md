# Mobile Robotics Project
Mobile Robotics Exam Project - University of Verona - A.Y. 2021/2022

## Purpose
The goal of this project is to improve the wall following algorithm developed in the lab, implementing feature extraction approaches from range data.
The case considered is the one where the robot must follow the walls on its right.
The Ransac algorithm was exploited to fit lines from the turtlebot3 lidar data, the fitted lines correspond to the environmental walls.
During the execution the robot will align itself with the fitted line and follow them.

## Documentation
### Definition of the lidar regions:

We maintaned the number and nomenclature of the regions used in the lab lecture: Front, Left, Right.

We changed how these region are constructed to improve the performance of the algorithm.
We reduced the size of Left and Right zone to make the robot more responsive in the case where it is following a wall which suddenly ends.
By exploiting the reduced regions, the robot will start turning right before w.r.t the case where the _behind_ region is considered.
From our tests we saw that the omitting  the data from the _behind_ region do not lead to a bad enough ransac line estimation to compromise the follow wall algorithm.

![alt text](/images/Mobile_Robotics_regions.drawio.png)

The computation of these regions was made parametrically to address real lidar sensors, since is not always the case that a lidar sensor have a 1:1 correspondence between angles and number of ranges. To clarify the previous statement, in simulation we had a 1:1 correspondance, 1 ray every 1 degree, on the tested turlebot3 the LaserScan message contained only 230 rays. 

### Description of the expanded FSM:

![alt text](/images/Mobile_robotics_FSM.drawio.png)

States:
* FIND_WALL: Initial state of the FSM. The robot will move to find a wall to follow, the angular velocity assign in this state was set at -0.6 m/s. To guarantee to always find a wall once the code is run, the angular velocity is continously decreased until 0.0 m/s at start. The linear velocity was set to 0.1 m/s.
* WAITING: In this state the robot will stay still for a little amount of _waitingCycles_, this was done to be able to work with clean lidar data
* FOLLOW_WALL: the robot will move ahed with a linear velocity of 0.1 m/s
* ALIGN_LEFT: the robot will try to fit lines from the acquired range data and align itself in the direction of the closest one. The angular velocity of this state was set to 0.5 m/s

State transtions:
* E1: If front region < _Th_ & (currentState == FIND_WALL || currentState == FOLLOW_WALL)
* E2: If waitingCycles <= 0 & currentState == WAITING
* E3: If currentState == ALIGN_LEFT & _aligned_
* E4: If right region > _Th_ & currentState == FOLLOW_WALL

The _aligned_ variable was used to indicate when the robot is aligned with the chosen line from the ransac algorithm, i.e. when the orientation of the computed line corresponds with the one of the robot

## Clarifications 
### Ransac
We extracted the 2d points from the LaserScan Message of the turtlebot3. From this set we applied the standard ransac algorithm to fit lines. 
In the case where the robot find itself in a corner, case detectable if both the mininum value of the right and front region are below our _treshold_, we discarded the points of the right region from the 2d points extracted from the LaserScan message. This was done to avoid fitting the wrong line corresponding to the right wall and insted be sure to fit the line corresponding to the front wall.
If the ransac execution does not resolve any good enough line, the number of inliers will be incrementaly reduced until at least one line is resolved

### ALIGN_LEFT State
The Align Left state consists of choosing the fitted ransac line closer to the robot and rotating left until the robot is aligned with the fitted line, with a certain margin of tolerance. Once the robot is aligned, the aligned variable is set to True to allow the change to the FOLLOW_WALL FSM state.

### Parameters
The code can accept a file where the parameters for the node are specified, if no file is provided, a set of default parameters is assumed.
A guideline for these file is `config/real_robot_params.yaml`, which contains the parameters used for the real turtlebot3.

**The tuning of these paramaters is essential for a good performance**

### Visualization
We exploit RViz to visualize the fitted lines.
By launching the awf.launch.py, you will be able to visualize in RViz the cloud of points of the robot's lidar sensor, the cloud of points corresponding to the fitted ransac line, and the robot position

## Steps for running the code
### ~/.bashrc
To be able to run the node on the real turtlebot you will have to add these lines on your `~/.bashrc`

```
export ROS_DOMAIN_ID = #IdOfTheTurtleBot3
export TURTLEBOT3_MODEL = #ModelOfTheTurtlebot3, burger or waffle
```

The ROS_DOMAIN_ID must be commented if you want to run the code in the simulation case, remember to `souce ~/.bashrc` after editing this file to apply the changes

### Simulation
* open Turtlebot3UnityROS2 project via Unity Hub
* visualizer - ros2 launch advanced_wall_following awf.launch.py 
* ros2 node  - ros2 run advanced_wall_following advanced_wall_following 

### Turtlebot3
* ssh ubuntu@<robot_ip> - pw: turtlebot
* ros2 launch turtlebot3_bringup robot.launch.py
* visualizer - ros2 launch advanced_wall_following awf.launch.py 
* ros2 node  - ros2 run advanced_wall_following advanced_wall_following 

### Run with parameter file
* ros2 run advanced_wall_following advanced_wall_following advanced_wall_following_node --ros-args --params-file src/advanced_wall_following/config/real_robot_params.yaml
