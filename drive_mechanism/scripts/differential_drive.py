#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity

from __future__ import division
import rospy
from can_interface.msg import WheelRPM
from geometry_msgs.msg import Twist

class DifferentialDrive:
    radian_per_sec_to_rpm = 9.54929658551

    def __init__(self):
        self.wheel_sep = rospy.get_param('~wheel_sep')
        self.wheel_radius = rospy.get_param('~wheel_radius')

        self.wheel_rpm_publisher = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=1)

        rospy.Subscriber('/robot_velocity_commmand', Twist, self.publish_wheel_rpm, queue_size=1)

    def publish_wheel_rpm(self, twist):
        robot_v = twist.linear.x
        robot_w = twist.angular.z

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

if __name__ ==  '__main__':
    node = rospy.init_node('drive_mechanism')

    differential_drive = DifferentialDrive()

    rospy.spin()