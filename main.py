#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep
from enum import Enum

class Command(Enum):
    MOVE_STRAIGHT = 1
    STOP = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    TURN_CENTER = 5
    START = 6
    ACTIVATE_CAMERA = 7

class MyRobot():
    def __init__(self):
        rospy.init_node('my_robot_node')
        self.euler = 0
        self.rate = rospy.Rate(10)
        self.state = Command.START

        # Publisher and Subscriber Initialization
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        self.sub_odo = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odometry, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser)
        self.pub_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.v = Twist()

        # Laser Readings
        self.center = 10
        self.right = 0
        self.left = 0

    def move_head(self, direction):
        if direction == Command.TURN_LEFT:
            x = 2
            y = 0
            self.state = Command.TURN_LEFT
        elif direction == Command.TURN_RIGHT:
            x = -2
            y = 0
            self.state = Command.TURN_RIGHT
        elif direction == Command.TURN_CENTER:
            x = 0
            y = 0
            self.state = Command.TURN_CENTER
        else:
            return

        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [x, y]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        self.head_pub.publish(msg)
        self.rate.sleep()

    def callback_odometry(self, msg):
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quaternion)[2]

    def callback_laser(self, msg):
        self.left = msg.ranges[len(msg.ranges) - 61]
        self.right = msg.ranges[61]
        self.center = msg.ranges[len(msg.ranges) // 2]

    def move_straight(self):
        self.v.linear.x = 0.3
        self.pub_vel.publish(self.v)

    def stop(self):
        self.v.linear.x = 0
        self.pub_vel.publish(self.v)

    def turn(self, direction):
        if direction == Command.TURN_RIGHT:
            orientation = self.euler - 1.57
        elif direction == Command.TURN_LEFT:
            orientation = self.euler + 1.57
        else:
            orientation = 0.0

        self.state = Command.TURNING
        while abs(self.euler - orientation) > 0.1:
            self.v.angular.z = -0.5 if direction == Command.TURN_RIGHT else 0.5
            self.pub_vel.publish(self.v)
            self.rate.sleep()

        self.v.angular.z = 0.0
        self.pub_vel.publish(self.v)
        self.state = Command.STOP

    def rotate_to_zero_degree(self):
        orientation = 0.0
        print(self.euler - orientation)
        while abs(self.euler - orientation) > 0.01:
            if self.euler > orientation:
                self.v.angular.z = -0.1
                self.pub_vel.publish(self.v)
                self.rate.sleep()
            else:
                self.v.angular.z = 0.1
                self.pub_vel.publish(self.v)
                self.rate.sleep()

        self.v.angular.z = 0.0
        self.pub_vel.publish(self.v)
        self.state = Command.STOP

    def decision(self):
        if self.state == Command.START:
            sleep(2)
            if self.euler != 0.000:
                self.rotate_to_zero_degree()
            self.state = Command.MOVE_STRAIGHT

        if self.state == Command.MOVE_STRAIGHT:
            if self.center > 0.8:
                self.move_straight()
            else:
                self.stop()
                self.state = Command.STOP

        if self.state == Command.STOP:
            if self.left > 1.5:
                self.state = Command.TURN_LEFT
            elif self.right > 1.5:
                self.state = Command.TURN_RIGHT
            elif self.left > 1.5 and self.right > 1.5:
                self.state = Command.ACTIVATE_CAMERA

        if self.state == Command.TURN_LEFT:
            self.turn(Command.TURN_LEFT)

        if self.state == Command.TURN_RIGHT:
            self.turn(Command.TURN_RIGHT)

        if self.state == Command.TURN_CENTER:
            self.turn(Command.TURN_CENTER)

        if self.state == Command.ACTIVATE_CAMERA:
            self.move_head(Command.TURN_LEFT)
            sleep(3)
            self.move_head(Command.TURN_RIGHT)
            sleep(3)
            self.move_head(Command.TURN_CENTER)
            sleep(3)

if __name__ == '__main__':
    tiago = MyRobot()
    while not rospy.is_shutdown():
        tiago.decision()
        tiago.rate.sleep()
