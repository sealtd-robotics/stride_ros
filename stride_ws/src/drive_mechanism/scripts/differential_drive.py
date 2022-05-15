#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity

from __future__ import division
import queue
import rospy, time
from can_interface.msg import WheelRPM
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Pose2D
from shared_tools.overseer_states_constants import *

ACCEL_DEFAULT = 3.5 # m/s2
DECEL_DEFAULT = 3.5 # m/s2

class DifferentialDrive:
    radian_per_sec_to_rpm = 9.54929658551

    def __init__(self):
        self.wheel_sep = rospy.get_param('~wheel_sep')
        self.left_wheel_radius = rospy.get_param('~left_wheel_radius')
        self.right_wheel_radius = rospy.get_param('~right_wheel_radius')
        
        self.overseer_state = 0

        self.commanded_robot_v = 0
        self.commanded_robot_w = 0

        self.wheel_rpm_publisher = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=1)

        rospy.Subscriber('/robot_velocity_command', Pose2D, self.velocity_command_callback, queue_size=1)      
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

    def publish_wheel_rpm(self, robot_v, robot_w):
        # # if not in manual, auto, or descending, don't publish
        # if not (self.overseer_state == 1 or self.overseer_state == 2 or self.overseer_state == 6 ):
        #     return

        # Differential drive formulas
        left_wheel_w = (robot_v - robot_w*self.wheel_sep/2) / self.left_wheel_radius
        left_wheel_rpm = left_wheel_w * self.radian_per_sec_to_rpm

        right_wheel_w = (robot_v + robot_w*self.wheel_sep/2) / self.right_wheel_radius
        right_wheel_rpm = right_wheel_w * self.radian_per_sec_to_rpm

        msg = WheelRPM()
        msg.left_front = left_wheel_rpm
        msg.left_back = left_wheel_rpm
        msg.right_front = right_wheel_rpm
        msg.right_back = right_wheel_rpm

        self.wheel_rpm_publisher.publish(msg)

    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data

    def velocity_command_callback(self, pose2d):
        self.commanded_robot_v = pose2d.x
        self.commanded_robot_w = pose2d.theta

if __name__ ==  '__main__':
    node = rospy.init_node('drive_mechanism')

    df = DifferentialDrive()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        state = df.overseer_state

        if state == MANUAL or state == AUTO or state == DESCENDING:
            df.publish_wheel_rpm(df.commanded_robot_v, df.commanded_robot_w)
        else: 
            df.commanded_robot_v = 0
            df.commanded_robot_w = 0
            df.publish_wheel_rpm(0,0)

        rate.sleep()