import imp
import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
import math
import time
import threading
from advanced_wall_following.helpers import ransac

class Point(object):
    distance = 0
    angle = 0
    x = 0
    y = 0

    # The class "constructor" - It's actually an initializer
    def __init__(self, distance, angle, x, y):
        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y


class AdvancedWallFollowing(Node):

    def __init__(self):
        super().__init__('advance_wall_following_node')
        # definition of publisher and subscriber object to /cmd_vel and /scan
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)

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

        # distance threshold to the wall
        self.th = 0.15

        timer_period = 0.1  # seconds

        self.turnAround = 30

        self.wallToRight = False

        self.rewind = False
        self.msg_list_lin = []
        self.msg_list_ang = []

        self.scanPoints = []

        # self.timer = self.create_timer(timer_period, self.control_loop)

        # keyboard.on_press_key('r', self.onPressKeyCallback)

    # loop each 0.1 seconds

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

        self.publisher_.publish(self.msg)

    # laser scanner callback

    def laser_callback(self, msg):

        ranges = msg.ranges
        
        numOfRanges = len(ranges)

        # in this way it should also work when we test in real robot
        angBetwewn2Rays = 360/numOfRanges

        self.scanPoints.clear()
        for idx in range(numOfRanges):
            r = np.nan_to_num(ranges[idx])
            angle = idx*angBetwewn2Rays
            x = r*math.cos(angle)
            # elements in ranges start from angle 0 and ends with angle 360 clockwise, reason for the -
            y = -r*math.sin(angle)
            # p = Point(r, angle, x, y)
            self.scanPoints.append(np.array([x,y]))
            # print("IDX ",idx, " X ", x, " Y ",y)
        lines = list()
        nInliners = 80
        maxIter = 100
        threshold = 0.5
        points2fit = self.scanPoints
        while(True):
            line, B, C, inliers, outliers = ransac(points2fit, maxIter, threshold, nInliners)
            if line is None:
                break
            lines.append(line)
            if len(outliers) <= nInliners:
                break
            points2fit = outliers

        print(lines)

        # print("\n\nAAAAAAAAAAA\n\n")

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
