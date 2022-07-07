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
import keyboard

class AdvancedWallFollowing(Node):

    def __init__(self):
        super().__init__('advance_wall_following_node')
        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        
        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
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

        self.turnAround=30

        self.wallToRight=False

        self.rewind=False
        self.msg_list_lin=[]
        self.msg_list_ang=[]

        self.timer = self.create_timer(timer_period, self.control_loop)

        keyboard.on_press_key('r', self.onPressKeyCallback)

    
    def onPressKeyCallback(self,event):
        print('r key pressed')
        if(self.rewind):
            self.msg_list_lin=[]
            self.msg_list_ang=[]
            self.change_state(2)

        self.turnAround=30
        self.rewind = not self.rewind
        
        
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
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 

        ranges = msg.ranges
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


    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call change_state function with the state index to enable the change state

        if(self.rewind):
            self.change_state(4)
        else:    
            if(self.wallToRight):
                if self.regions['right'] > self.th and (self.state_ == 1 or self.state_ == 2) :
                    self.change_state(0)
                elif self.regions['front'] < self.th and (self.state_ == 0 or self.state_ == 2):
                    self.change_state(1)
                elif self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th and (self.state_ == 0 or self.state_ == 1):
                    self.change_state(2)   
            else:
                if self.regions['left'] > self.th and (self.state_ == 3 or self.state_ == 2) :
                    self.change_state(0)
                elif self.regions['front'] < self.th and (self.state_ == 0 or self.state_ == 2):
                    self.change_state(3)
                elif self.regions['front'] > self.th and self.regions['right'] > self.th and self.regions['left'] < self.th and (self.state_ == 0 or self.state_ == 3):
                    self.change_state(2) 
            

        
    # function to update state
    # don't modify the function
    def change_state(self, state):
        
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def find_wall(self):
        print("find")
        self.msg.linear.x  = 0.1
        if(self.wallToRight):
            self.msg.angular.z=-0.8
        else:
            self.msg.angular.z = 0.5

    def align_left(self):
        print("align left")
        self.msg.linear.x  = 0.0
        self.msg.angular.z = 1.0
    
    def align_right(self):
        print("align right")
        self.msg.linear.x  = 0.0
        self.msg.angular.z = -1.0

    def follow_the_wall(self):
        print("follow")
        self.msg.linear.x  = 0.1
        self.msg.angular.z = 0.0

    def reverse(self):
        if(self.turnAround>0):
            self.msg.linear.x  = 0.0
            self.msg.angular.z = 1.0
            self.turnAround-=1
            return

        if(len(self.msg_list_lin)==0):
            self.rewind=False
            self.change_state(2)
        else:
            self.msg.linear.x  = self.msg_list_lin[len(self.msg_list_lin)-1]
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
