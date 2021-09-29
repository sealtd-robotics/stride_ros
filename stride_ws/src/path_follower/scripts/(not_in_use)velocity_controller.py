#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity

from __future__ import division
import rospy
import math
import os
import time
from std_msgs.msg import Int32, Float32, String, Empty, Float32MultiArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from path_follower.msg import Latlong

from math import cos, sin, sqrt, pi, atan2
from glob import glob


class VelocityController:
    def __init__(self):
        self.overseer_state = 5      # 5: STOPPED state
        self.robot_heading = 0

        self.path_easts = []
        self.path_norths = []
        self.turning_radius = 99999
        self.desired_speed = 0
        self.path_intervals = []

        # Publishers
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_1)
        rospy.Subscriber('/robot_commander/desired_speed', Float32, self.subscriber_callback_3)
        rospy.Subscriber('/path_follower/path_intervals', Float32MultiArray, self.path_intervals_callback)
        rospy.Subscriber('/an_device/Twist', Twist, self.gps_twist_callback, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_subscriber_callback_2, queue_size=1)

    def publish_robot_velocity(self):
        max_index = len(self.path_easts) - 1

        if self.current_path_index == max_index:
            pose2d = Pose2D()
            pose2d.x = 0
            pose2d.theta = 0
            self.robot_velocity_publisher.publish(pose2d)
            return
        pose2d = Pose2D()
        pose2d.x = self.desired_speed
        pose2d.theta = self.desired_speed / self.turning_radius

        self.robot_velocity_publisher.publish(pose2d)

    # Subscriber callbacks
    def subscriber_callback_1(self, msg):
        self.overseer_state = msg.data

    def subscriber_callback_2(self, msg):
        self.load_path()

    def subscriber_callback_3(self, msg):
        self.desired_speed = msg.data

    def subscriber_callback_4(self, msg):
        self.stop_index = msg.data

    # GPS subscriber callbacks
    def gps_subscriber_callback_1(self, msg):
        (self.robot_north, self.robot_east)  = self.LL2NE(msg.latitude, msg.longitude)

    def gps_subscriber_callback_2(self, msg):
        self.robot_heading = msg.data    # radian

    # def gps_subscriber_callback_3(self, msg):
    #     self.linear_speed_measured = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
    #     self.yaw_velocity_measured = msg.angular.z

if __name__ ==  '__main__':
    node = rospy.init_node('path_follower/velocity_controller')

    vc = VelocityController()

    should_reset_variables = False
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if pf.overseer_state == 2:    # 2 is the autonomous (AUTO) state
            pf.update_current_path_index()
            pf.update_turning_radius()
            pf.publish_robot_velocity()
            should_reset_variables = True

        elif should_reset_variables:
            pf.current_path_index = 0
            pf.current_path_index_publisher.publish(0)
            pf.desired_speed = 0

            should_reset_variables = False

        rate.sleep() 
