#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity

from __future__ import division
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D

class DifferentialDrive:
    radian_per_sec_to_rpm = 9.54929658551

    def __init__(self):
        self.wheel_sep = rospy.get_param('~wheel_sep')
        self.wheel_radius = rospy.get_param('~wheel_radius')
        
        self.overseer_state = 0

        self.wheel_rpm_publisher = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=1)

        rospy.Subscriber('/robot_velocity_command', Pose2D, self.publish_wheel_rpm, queue_size=1)
        
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

    def publish_wheel_rpm(self, pose2d):
        # if not in either manual or auto, don't publish
        if not (self.overseer_state == 1 or self.overseer_state == 2):
            return

        robot_v = pose2d.x
        robot_w = pose2d.theta

        left_wheel_w = (robot_v - robot_w*self.wheel_sep/2) / self.wheel_radius
        left_wheel_rpm = left_wheel_w * self.radian_per_sec_to_rpm

        right_wheel_w = (robot_v + robot_w*self.wheel_sep/2) / self.wheel_radius
        right_wheel_rpm = right_wheel_w * self.radian_per_sec_to_rpm

        msg = WheelRPM()
        msg.left_front = left_wheel_rpm
        msg.left_back = left_wheel_rpm
        msg.right_front = right_wheel_rpm
        msg.right_back = right_wheel_rpm

        self.wheel_rpm_publisher.publish(msg)

    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data


if __name__ ==  '__main__':
    node = rospy.init_node('drive_mechanism')

    df = DifferentialDrive()

    zero_rpm = WheelRPM()
    zero_rpm.left_front = 0
    zero_rpm.left_back = 0
    zero_rpm.right_front = 0
    zero_rpm.right_back = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # if not in either manual or auto, publish zero rpm
        if not (df.overseer_state == 1 or df.overseer_state == 2):
            df.wheel_rpm_publisher.publish(zero_rpm)
        rate.sleep()