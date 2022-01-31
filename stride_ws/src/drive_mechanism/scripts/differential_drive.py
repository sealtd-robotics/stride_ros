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

ACCEL_DEFAULT = 3.5 # m/s2
DECEL_DEFAULT = 3.5 # m/s2

class DifferentialDrive:
    radian_per_sec_to_rpm = 9.54929658551

    def __init__(self):
        self.wheel_sep = rospy.get_param('~wheel_sep')
        self.wheel_radius = rospy.get_param('~wheel_radius')
        
        self.overseer_state = 0

        self.desired_robot_v = 0
        self.limited_robot_v = 0
        self.desired_robot_w = 0
        self.limited_robot_w = 0
        self.turning_radius = 999999
        
        self.path_acceleration = ACCEL_DEFAULT 
        self.path_deceleration = DECEL_DEFAULT
        self.previous_time = time.time()

        self.wheel_rpm_publisher = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=1)

        rospy.Subscriber('/robot_velocity_command', Pose2D, self.velocity_command_callback, queue_size=1)
        rospy.Subscriber('/robot_acceleration_command_for_path_following', Float32, self.acceleration_callback, queue_size=1)
        rospy.Subscriber('/robot_deceleration_command_for_path_following', Float32, self.deceleration_callback, queue_size=1)
        
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

    def set_limited_velocity(self, acceleration, deceleration):
        time_now = time.time()
        dt = time_now - self.previous_time
        self.previous_time = time_now

        if self.desired_robot_v > self.limited_robot_v:
            self.limited_robot_v = min(self.desired_robot_v, self.limited_robot_v + acceleration * dt)
            self.limited_robot_w = self.limited_robot_v / self.turning_radius
        else:
            self.limited_robot_v = max(self.desired_robot_v, self.limited_robot_v - deceleration * dt)
            self.limited_robot_w = self.limited_robot_v / self.turning_radius

    def publish_wheel_rpm(self, robot_v, robot_w):
        # # if not in manual, auto, or decent, don't publish
        # if not (self.overseer_state == 1 or self.overseer_state == 2 or self.overseer_state == 6 ):
        #     return

        # Differential drive formulas
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

    def velocity_command_callback(self, pose2d):
        self.desired_robot_v = pose2d.x
        self.desired_robot_w = pose2d.theta

        # needed for calculating the limited_robot_w after taking acceleration or deceleration into consideration
        if pose2d.theta == 0:
            self.turning_radius = 999999
        else:
            self.turning_radius = pose2d.x / pose2d.theta
            

    def acceleration_callback(self, msg):
        self.path_acceleration = abs(msg.data)

    def deceleration_callback(self, msg):
        self.path_deceleration = abs(msg.data)



if __name__ ==  '__main__':
    node = rospy.init_node('drive_mechanism')

    df = DifferentialDrive()

    rate = rospy.Rate(50)

    previous_state = 0
    state = 0
    while not rospy.is_shutdown():
        state = df.overseer_state

        # if not either manual, auto, or decent, publish zero rpm
        # needed because the above states don't always publish zero before transitioning out. 
        if not (state == 1 or state == 2 or state == 6):
            df.publish_wheel_rpm(0,0)
        elif state == 1:
            # On entering this state
            if state != previous_state:
                df.desired_robot_v = 0
                df.desired_robot_w = 0

            df.publish_wheel_rpm(df.desired_robot_v, df.desired_robot_w)
        elif state == 2:
            # On entering this state
            if state != previous_state:
                df.limited_robot_v = 0
                df.previous_time = time.time()
            
            df.set_limited_velocity(df.path_acceleration, df.path_deceleration)
            df.publish_wheel_rpm(df.limited_robot_v, df.limited_robot_w)
        elif state == 6:
            # On entering this state
            if state != previous_state:
                df.limited_robot_v = 0
                df.previous_time = time.time()

            df.set_limited_velocity(0.05, 0.05)
            df.publish_wheel_rpm(df.limited_robot_v, df.limited_robot_w)

        # Reset accel and decel for path following
        if state != 2:
            df.path_deceleration = ACCEL_DEFAULT
            df.path_deceleration = DECEL_DEFAULT

        previous_state = state
        rate.sleep()