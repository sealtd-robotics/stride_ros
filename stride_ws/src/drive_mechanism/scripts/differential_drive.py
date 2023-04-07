#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity

from __future__ import division
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
        self.wheel_sep = rospy.get_param('wheel_sep')
        self.left_wheel_radius = rospy.get_param('left_wheel_radius')
        self.right_wheel_radius = rospy.get_param('right_wheel_radius')
        self.k_l = rospy.get_param('k_l')
        self.k_r = rospy.get_param('k_r')
        self.P_corr = rospy.get_param('P_corr')
        
        self.overseer_state = 0

        self.commanded_robot_v = 0
        self.commanded_robot_w = 0
        self.cross_track_error = 0

        self.wheel_rpm_publisher = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=1)

        rospy.Subscriber('/robot_velocity_command', Pose2D, self.velocity_command_callback, queue_size=1)      
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)
        rospy.Subscriber('/path_follower/cross_track_error', Float32, self.cross_track_error_callback, queue_size=1)

    def publish_wheel_rpm(self, robot_v, robot_w):
        #Check what side of path robot is on and adjust wheel RPM
        adj_L = 0
        adj_R = 0

        # No adjdustments when robot is not moving. Otherwise robot moves when executing sleep command.
        if not (robot_v == 0):
            if self.cross_track_error >= 0: #If on left side of path, add correction term for left wheel
                adj_L = self.P_corr * abs(self.cross_track_error)
                adj_R = -1 * self.P_corr * abs(self.cross_track_error)

            elif self.cross_track_error < 0: #If on right side of path, add correction term for right wheel
                adj_R = self.P_corr * abs(self.cross_track_error)
                adj_L = -1 * self.P_corr * abs(self.cross_track_error)

        # Differential drive formulas
        left_wheel_w = (robot_v - robot_w*self.wheel_sep/2) / (self.k_l * self.left_wheel_radius) + adj_L
        left_wheel_rpm = left_wheel_w * self.radian_per_sec_to_rpm
        right_wheel_w = (robot_v + robot_w*self.wheel_sep/2) / (self.k_r * self.right_wheel_radius) + adj_R
        right_wheel_rpm = right_wheel_w * self.radian_per_sec_to_rpm

        msg = WheelRPM()
        msg.left_front = left_wheel_rpm
        msg.left_back = left_wheel_rpm
        msg.right_front = right_wheel_rpm
        msg.right_back = right_wheel_rpm
        msg.adj_left = adj_L*self.radian_per_sec_to_rpm
        msg.adj_right = adj_R*self.radian_per_sec_to_rpm

        self.wheel_rpm_publisher.publish(msg)

    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data

    def velocity_command_callback(self, pose2d):
        self.commanded_robot_v = pose2d.x
        self.commanded_robot_w = pose2d.theta

    def cross_track_error_callback(self, msg):
        self.cross_track_error = msg.data

if __name__ ==  '__main__':
    node = rospy.init_node('drive_mechanism')

    df = DifferentialDrive()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        state = df.overseer_state

        if state == MANUAL or state == AUTO or state == DESCENDING or state == RETURN_TO_START:
            df.publish_wheel_rpm(df.commanded_robot_v, df.commanded_robot_w)
        else: 
            df.commanded_robot_v = 0
            df.commanded_robot_w = 0
            df.publish_wheel_rpm(0,0)
            df.cross_track_error = 0

        rate.sleep()