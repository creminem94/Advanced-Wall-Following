import math
import random

import numpy as np
import rclpy
import rclpy.qos
import tf_transformations
from geometry_msgs.msg import Point, Point32, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, LaserScan

from advanced_wall_following.helpers import point2lineDist, ransac
from advanced_wall_following.helpers import FsmState


class AdvancedWallFollowing(Node):

    def __init__(self):
        super().__init__('advance_wall_following_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('wall_threshold', 0.15),
                ('front_angle', 30),
                ('ransac.start_inliers', 25),
                ('ransac.inliers_decrease_factor', 3),
                ('ransac.max_iter', 100),
                ('ransac.point_distance_threshold', 0.01),
                ('find_wall.linear_vel', 0.1),
                ('find_wall.angular_vel', -0.6),
                ('find_wall.slow_down_factor', 0.005),
                ('align_left.angular_vel', 0.5),
                ('align_left.margin_angle', 0.03),
                ('align_left.aligned_threshold', 0.05),
                ('follow_wall.linear_vel', 0.1),
                ('waiting_cycles', 3)
            ]
        )

        # definition of publisher and subscriber object to /cmd_vel, /scan and /odom
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.publisher_ransac_lines = self.create_publisher(
            PointCloud, '/ransac_lines', 1)
        self.subscription_laser = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, rclpy.qos.qos_profile_sensor_data)

        # initial state of FSM
        self.previousState = FsmState.FIND_WALL
        self.currentState = FsmState.FIND_WALL

        # initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # velocity command
        self.msg = Twist()
        self.line_msg = PointCloud()

        # distance threshold to the wall
        self.th = self.get_parameter('wall_threshold').value
        self.timer_period = 0.1  # seconds

        self.frontAngle = self.get_parameter('front_angle').value
        self.frontLimit = 0
        self.aligned = False
        self.waitingCycles = 0

        self.scanPoints = []
        self.ransacLineParams = []

        # robot pose and orientation w.r.t. world frame, robotOrientation is in radiants
        self.robotPose = Point()
        self.robotOrientation = Quaternion()
        # current robot orientation w.r.t. /odom
        self.robotAngle = 0
        # robot angle just before starting the align procedure
        self.startingRobotAngle = 0
        self.targetAngle = None
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):

        # actions for states
        if self.currentState == FsmState.FIND_WALL:
            self.find_wall()
        elif self.currentState == FsmState.ALIGN_LEFT:
            self.align_left()
        elif self.currentState == FsmState.FOLLOW_WALL:
            self.follow_the_wall()
        elif self.currentState == FsmState.WAITING:
            self.waiting()
        else:
            print("Unknown state")

        self.publisher_cmd_vel.publish(self.msg)

    def laser_callback(self, msg):
        ranges = np.nan_to_num(msg.ranges, nan=100)

        numOfRanges = len(ranges)

        # zone's parametrization, in this code works also on real robot

        self.scanPoints = []
        for idx in range(numOfRanges):
            # for simulating noise on range data
            # ranges[idx] += random.randrange(-10, 10, 1)/10000
            r = np.nan_to_num(ranges[idx])
            angle = idx*msg.angle_increment
            x = r*math.cos(angle)
            # elements in ranges start from angle 0 and ends with angle 360 clockwise
            y = r*math.sin(angle)
            self.scanPoints.append(np.array([x, y]))
            # print("IDX ",idx, " X ", x, " Y ",y)
        half = int(len(ranges)/2)
        self.frontLimit = int(numOfRanges*self.frontAngle/360)
        rangesTopRight = ranges[0: self.frontLimit]
        rangesTopLeft = ranges[len(ranges) - self.frontLimit:len(ranges)-1]

        rangesRight = ranges[half + self.frontLimit *
                             2:len(ranges) - self.frontLimit]
        rangesLeft = ranges[self.frontLimit+1:half - self.frontLimit*2]
        self.regions = {
            'front':  min(min(min(rangesTopLeft), 10), min(min(rangesTopRight), 10)),
            'left':  min(min(rangesLeft), 10),
            'right':  min(min(rangesRight), 10),
        }
        # print(self.regions)
        # function where are definied the rules for the change state
        self.take_action()
        self.fitRansacLines(self.get_parameter('ransac.start_inliers').value)

    def fitRansacLines(self, nInliers):

        # contains list of pair of points that identify each line
        if self.currentState != FsmState.WAITING or nInliers <= 0:
            return
        maxIter = self.get_parameter('ransac.max_iter').value
        threshold = self.get_parameter('ransac.point_distance_threshold').value
        points2fit = self.scanPoints
        # print('regions', self.regions)
        excludeTh = self.th
        if self.regions['front'] < excludeTh and self.regions['right'] < self.th:
            print("exlude right data")
            # we must turn, so to avoid fitting the "right" line, we exclude the right points
            half = int(len(points2fit)/2)
            rangesTopRight = points2fit[0: self.frontLimit]
            rangesTopLeft = points2fit[len(
                points2fit) - self.frontLimit:len(points2fit)-1]
            rangesLeft = points2fit[self.frontLimit+1:half - self.frontLimit]
            points2fit = [*rangesTopRight, *rangesTopLeft, *rangesLeft]

        self.ransacLineParams = []
        while(True):
            line, B, C, inliers, outliers = ransac(
                points2fit, maxIter, threshold, nInliers)
            if line is None:
                break
            self.ransacLineParams.append({
                "p1": B,
                "p2": C,
                "m": line[0],
                "q": line[1],
                "inliers": inliers
            })
            if len(outliers) <= nInliers:
                break
            points2fit = outliers
        if len(self.ransacLineParams) == 0:
            decreasedInliers = nInliers - \
                self.get_parameter('ransac.inliers_decrease_factor').value
            print('Decreased inliers to: ', decreasedInliers)
            self.fitRansacLines(decreasedInliers)

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
        self.robotAngle = (yaw if yaw >= 0 else 6.28 + yaw)
        # print("odom ", yaw)

    def publish_line(self, ransacLine):
        inliers = ransacLine["inliers"]
        points = []
        for idx in range(len(inliers)):
            point = Point32()
            point.x = inliers[idx][0]
            point.y = inliers[idx][1]
            point.z = 0.01
            points.append(point)

        self.line_msg.header.stamp = self.get_clock().now().to_msg()
        # frame id must be /base_scan since is the one to which the laser scan data are referred to
        self.line_msg.header.frame_id = "/base_scan"
        self.line_msg.points = points

        self.publisher_ransac_lines.publish(self.line_msg)

    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call changeState function with the state index to enable the change state
        if self.regions['right'] > self.th and (self.currentState == FsmState.FOLLOW_WALL):
            self.changeState(FsmState.FIND_WALL)
        elif self.regions['front'] < self.th and (self.currentState == FsmState.FIND_WALL or self.currentState == FsmState.FOLLOW_WALL):
            self.waitingCycles = self.get_parameter('waiting_cycles').value
            self.changeState(FsmState.WAITING)
        elif self.waitingCycles <= 0 and self.currentState == FsmState.WAITING:
            self.startingRobotAngle = self.robotAngle
            self.changeState(FsmState.ALIGN_LEFT)
        elif self.currentState == FsmState.ALIGN_LEFT and self.aligned:
            self.aligned = False
            self.changeState(FsmState.FOLLOW_WALL)

    # function to update state
    def changeState(self, state):

        if state is not self.currentState:
            print('Wall follower - %s' % (state.name))
            self.previousState = self.currentState
            self.currentState = state

    # TODO finish setting up params

    def find_wall(self):
        print("find")
        self.msg.linear.x = 0.1
        minDist = min(self.regions.values())
        print("dist", minDist)
        if minDist > 0 and self.previousState == self.currentState:
            newVel = self.msg.angular.z + 0.005
            self.msg.angular.z = min(0.0, newVel)
        else:
            self.msg.angular.z = -0.6

    def align_left(self):
        print("align left")
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        if self.targetAngle:
            aligned, diff = self.checkIfAligned()
            if not aligned:
                self.msg.angular.z = 0.5
            else:
                self.aligned = True
                self.targetAngle = None
        elif len(self.ransacLineParams) > 0:
            # compute target angle
            distances = list(map(lambda p: point2lineDist(
                [0, 0], p["p1"], p["p2"]), self.ransacLineParams))
            closestIdx = np.argmin(distances)
            closestLine = self.ransacLineParams[closestIdx]
            self.publish_line(closestLine)

            # step 2: compute angle to align with line
            p1 = closestLine["p1"]
            p2 = closestLine["p2"]
            m = (p2[1]-p1[1])/(p2[0]-p1[0])
            angle = abs(np.arctan(m))
            marginAngle = 0.03
            self.targetAngle = (
                angle + self.startingRobotAngle + marginAngle) % 6.28
            # print("computed angle ", angle)
            # print("target angle", self.targgetAngle)
        else:
            print("no line fitted")

    def checkIfAligned(self):
        angle_th = 0.05
        # check if we've reached the target angle which indicates we're aligned with ransac line
        diff = abs(self.targetAngle - self.robotAngle)
        # print("check ", np.rad2deg(self.targetAngle),
        #       np.rad2deg(self.robotAngle), np.rad2deg(diff))
        return diff < angle_th or 6.28 - diff < angle_th, self.targetAngle - self.robotAngle

    def follow_the_wall(self):
        print("follow")
        self.msg.linear.x = 0.1
        self.msg.angular.z = 0.0

    def waiting(self):
        print("waiting")
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.waitingCycles -= 1


def main(args=None):
    print("main")
    rclpy.init(args=args)

    advance_wall_following_node = AdvancedWallFollowing()

    rclpy.spin(advance_wall_following_node)

    advance_wall_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
