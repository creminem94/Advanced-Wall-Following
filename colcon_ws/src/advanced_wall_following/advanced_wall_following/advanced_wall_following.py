import imp
import math
import threading
import time

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

from advanced_wall_following.helpers import ransac

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
        self.state_ = 0

        # initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # definition of dict with state of FSM
        self.state_dict_ = {
            0: 'find the wall',
            1: 'align left',
            2: 'follow the wall',
            3: 'align left',
            4: 'rewind',
        }
        # velocity command
        self.msg = Twist()
        self.line_msg = Path()

        # distance threshold to the wall
        self.th = 0.15

        timer_period = 0.1  # seconds

        self.turnAround = 30

        self.wallToRight = False

        self.rewind = False
        self.msg_list_lin = []
        self.msg_list_ang = []

        self.scanPoints = []

        # robot pose and orientation w.r.t. world frame, robotOrientation is in radiants
        self.robotPose = Point()
        self.robotOrientation = Quaternion()
        self.robotAngle = 0

        # self.timer = self.create_timer(timer_period, self.control_loop)

    def control_loop(self):

        # actions for states
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
        elif self.state_ == 3:
            self.align_right()
        elif self.state_ == 4:
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
        while(True):
            line, B, C, inliers, outliers = ransac(
                points2fit, maxIter, threshold, nInliners)
            if line is None:
                break
            self.publish_line([B,C])
            # lines.append(line)
            if len(outliers) <= nInliners:
                break
            points2fit = outliers
        # print(lines)

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
        # call change_state function with the state index to enable the change state

        if(self.rewind):
            self.change_state(4)
        else:
            if(self.wallToRight):
                if self.regions['right'] > self.th and (self.state_ == 1 or self.state_ == 2):
                    self.change_state(0)
                elif self.regions['front'] < self.th and (self.state_ == 0 or self.state_ == 2):
                    self.change_state(1)
                elif self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th and (self.state_ == 0 or self.state_ == 1):
                    self.change_state(2)
            else:
                if self.regions['left'] > self.th and (self.state_ == 3 or self.state_ == 2):
                    self.change_state(0)
                elif self.regions['front'] < self.th and (self.state_ == 0 or self.state_ == 2):
                    self.change_state(3)
                elif self.regions['front'] > self.th and self.regions['right'] > self.th and self.regions['left'] < self.th and (self.state_ == 0 or self.state_ == 3):
                    self.change_state(2)

    # function to update state
    # don't modify the function

    def change_state(self, state):

        if state is not self.state_:
            print('Wall follower - [%s] - %s' %
                  (state, self.state_dict_[state]))
            self.state_ = state

    def find_wall(self):
        print("find")
        self.msg.linear.x = 0.1
        if(self.wallToRight):
            self.msg.angular.z = -0.8
        else:
            self.msg.angular.z = 0.5

    def align_left(self):
        print("align left")
        self.msg.linear.x = 0.0
        self.msg.angular.z = 1.0

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
            self.change_state(2)
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
