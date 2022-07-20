import imp
import math
import threading
import time
from turtle import distance
from matplotlib.pyplot import close

import numpy as np
import rclpy
import rclpy.qos
import tf2_py
import tf_transformations
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from advanced_wall_following.helpers import point2lineDist, ransac
from advanced_wall_following.helpers import FsmState

class AdvancedWallFollowing(Node):  

    def __init__(self):
        super().__init__('advance_wall_following_node')
        # definition of publisher and subscriber object to /cmd_vel, /scan and /odom
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.publisher_ransac_lines = self.create_publisher(
            Path, '/ransac_lines', 1)
        self.subscription_laser = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, rclpy.qos.qos_profile_sensor_data)

        # initial state of FSM
        self.currentState = FsmState.FIND_WALL

        # initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # velocity command
        self.msg = Twist()
        self.line_msg = Path()

        # distance threshold to the wall
        self.th = 0.15

        self.timer_period = 0.1  # seconds

        self.turnAround = 30
        self.nCycles = 0
        self.aligned = False

        self.wallToRight = True

        self.rewind = False
        self.msg_list_lin = []
        self.msg_list_ang = []

        self.scanPoints = []
        self.ransacLineParams = []

        # robot pose and orientation w.r.t. world frame, robotOrientation is in radiants
        self.robotPose = Point()
        self.robotOrientation = Quaternion()
        self.robotAngle = 0

        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):

        # actions for states
        if self.currentState == FsmState.FIND_WALL:
            self.find_wall()
        elif self.currentState == FsmState.ALIGN_LEFT:
            self.align_left()
        elif self.currentState == FsmState.FOLLOW_WALL:
            self.follow_the_wall()
        elif self.currentState == FsmState.ALIGN_RIGHT:
            self.align_right()
        elif self.currentState == FsmState.REWIND:
            self.reverse()
            pass
        else:
            print("Unknown state")

        if(not self.rewind):
            self.msg_list_lin.append(self.msg.linear.x)
            self.msg_list_ang.append(self.msg.angular.z)

        self.publisher_cmd_vel.publish(self.msg)

    # laser scanner callback

    def laser_callback(self, msg):

        ranges = msg.ranges

        numOfRanges = len(ranges)

        # in this way it should also work when we test in real robot
    
        self.scanPoints=[]
        for idx in range(numOfRanges):
            r = np.nan_to_num(ranges[idx])
            angle = idx*msg.angle_increment
            x = r*math.cos(angle)
            # elements in ranges start from angle 0 and ends with angle 360 clockwise, reason for the -
            y = r*math.sin(angle)
            # p = Point(r, angle, x, y)
            self.scanPoints.append(np.array([x, y]))
            # print("IDX ",idx, " X ", x, " Y ",y)
    
        nInliners = 25
        maxIter = 100
        threshold = 0.001
        points2fit = self.scanPoints
        #contains list of pair of points that identify each line
        self.ransacLineParams = []
        while(True):
            line, B, C, inliers, outliers = ransac(
                points2fit, maxIter, threshold, nInliners)
            if line is None:
                break
            self.ransacLineParams.append({
                "p1": B,
                "p2": C,
                "m": line[0],
                "q": line[1]
            })
            # self.publish_line([B,C])
            # lines.append(line)
            if len(outliers) <= nInliners:
                break
            points2fit = outliers
        # print(lines)

        frontAngle = 60
        half = int(len(ranges)/2)
        rangesTopRight = ranges[0:frontAngle]
        rangesTopLeft = ranges[len(ranges)-frontAngle:len(ranges)-1]
        rangesRight = ranges[half+frontAngle:len(ranges)-frontAngle]
        rangesLeft = ranges[frontAngle+1:half-frontAngle]
        self.regions = {
            'front':  min(min(min(rangesTopLeft), 10), min(min(rangesTopRight), 10)),
            'left':  min(min(rangesLeft), 10),
            'right':  min(min(rangesRight), 10),
        }
        # function where are definied the rules for the change state
        self.take_action()

    def odom_callback(self, msg):
        # Estimated pose that is typically relative to the fixed world frame.
        pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        orientation_list = [orientation.x,
                            orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # Angle between world frame and robot frame is in radiants
        self.robotPose = pose
        self.robotOrientation = orientation
        self.robotAngle = yaw

    def publish_line(self, points):
        poses=[]
        for idx in range(len(points)):
            pose = PoseStamped()
            pose.pose.position.x = points[idx][0]
            pose.pose.position.y = points[idx][1]
            pose.pose.position.z = 0.01
            pose.header.stamp=self.get_clock().now().to_msg()
            pose.header.frame_id="/pose"

            poses.append(pose)
            
        self.line_msg.header.stamp=self.get_clock().now().to_msg()
        # frame id must be /base_scan since is the one to which the laser scan data are referred to
        self.line_msg.header.frame_id="/base_scan"
        self.line_msg.poses = poses
        
        self.publisher_ransac_lines.publish(self.line_msg)

    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call changeState function with the state index to enable the change state
        # if (self.currentState != FsmState.ALIGN_LEFT):
            if self.regions['right'] > self.th and (self.currentState == FsmState.FOLLOW_WALL):
                self.changeState(FsmState.FIND_WALL)
            elif self.regions['front'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.FOLLOW_WALL):
                self.changeState(FsmState.ALIGN_LEFT)
            elif self.currentState == FsmState.ALIGN_LEFT and self.aligned:
                self.aligned = False
                self.changeState(FsmState.FOLLOW_WALL)
            # elif self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.ALIGN_LEFT):
            #     self.changeState(FsmState.FOLLOW_WALL)


        # if(self.rewind ):
        #     self.changeState(FsmState.REWIND)
        # else:
        #     if(self.wallToRight):
        #         if self.regions['right'] > self.th and (self.currentState == FsmState.ALIGN_LEFT or self.currentState == FsmState.FOLLOW_WALL):
        #             self.changeState(FsmState.FIND_WALL)
        #         elif self.regions['front'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.FOLLOW_WALL):
        #             self.changeState(FsmState.ALIGN_LEFT)
        #         elif self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.ALIGN_LEFT):
        #             self.changeState(FsmState.FOLLOW_WALL)
        #     else:
        #         if self.regions['left'] > self.th and (self.currentState == FsmState.ALIGN_RIGHT or self.currentState == FsmState.FOLLOW_WALL):
        #             self.changeState(FsmState.FIND_WALL)
        #         elif self.regions['front'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.FOLLOW_WALL):
        #             self.changeState(FsmState.ALIGN_RIGHT)
        #         elif self.regions['front'] > self.th and self.regions['right'] > self.th and self.regions['left'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.ALIGN_RIGHT):
        #             self.changeState(FsmState.FOLLOW_WALL)
        
    # function to update state
    # don't modify the function

    def changeState(self, state):

        if state is not self.currentState:
            print('Wall follower - %s' % (state.name))
            self.currentState = state

    def find_wall(self):
        print("find")
        self.msg.linear.x = 0.1
        if(self.wallToRight):
            self.msg.angular.z = -0.8
        else:
            self.msg.angular.z = 0.5

    def align_left(self):
        print("align left")
        vel = 1.0
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        if (self.nCycles > 0):
            self.msg.linear.x = 0.0
            self.msg.angular.z = vel
            self.nCycles -= 1
            if (self.nCycles == 0):
                self.aligned = True
            #     self.changeState(FsmState.FOLLOW_WALL)
            return
        if self.aligned:
            return
        #step 1: closest ransac line to robot
        distances = list(map(lambda p: point2lineDist([0,0], p["p1"], p["p2"]), self.ransacLineParams))
        closestIdx = np.argmin(distances)
        closestLine = self.ransacLineParams[closestIdx]
        self.publish_line([closestLine["p1"], closestLine["p2"]])

        #step 2: compute angle to align with line
        p1 = closestLine["p1"]
        p2 = closestLine["p2"]
        m = (p2[1]-p1[1])/(p2[0]-p1[0])
        angle = abs(np.arctan(m))
        print("computed angle ", angle)

        #step 3: compute number of cycles to complete rotation
        timeInSec = angle/vel
        self.nCycles = np.ceil(timeInSec/self.timer_period)


        

    def align_right(self):
        print("align right")
        self.msg.linear.x = 0.0
        self.msg.angular.z = -1.0

    def follow_the_wall(self):
        print("follow")
        self.msg.linear.x = 0.1
        self.msg.angular.z = 0.0

    def reverse(self):
        if(self.turnAround > 0):
            self.msg.linear.x = 0.0
            self.msg.angular.z = 1.0
            self.turnAround -= 1
            return

        if(len(self.msg_list_lin) == 0):
            self.rewind = False
            self.changeState(FsmState.FOLLOW_WALL)
        else:
            self.msg.linear.x = self.msg_list_lin[len(self.msg_list_lin)-1]
            self.msg.angular.z = -self.msg_list_ang[len(self.msg_list_ang)-1]
            self.msg_list_lin.pop()
            self.msg_list_ang.pop()
            print("rewind msg: ", self.msg)


def main(args=None):
    print("main")
    rclpy.init(args=args)

    advance_wall_following_node = AdvancedWallFollowing()

    rclpy.spin(advance_wall_following_node)

    advance_wall_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
