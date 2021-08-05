#!/usr/bin/env python

from __future__ import division
import rospy
import time
import threading
import os
import sys

from std_msgs.msg import Int32, Empty, Bool
from geometry_msgs.msg import Pose2D
from datetime import datetime

class RobotCommander:
    def __init__(self):
        self.process = None
        self.is_running = False

        # Publishers
        self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)
    
    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data

    def get_time_now_in_ms(self):
        epoch = datetime.utcfromtimestamp(0)
        now = datetime.utcnow()
        delta = now - epoch
        return delta.total_seconds() * 1000

    def go_straight_for_milliseconds(self, speed, milliseconds):
        start_time = self.get_time_now_in_ms()
        pose2d = Pose2D()
        pose2d.x = speed
        pose2d.theta = 0

        rate = rospy.Rate(10)
        while self.get_time_now_in_ms() - start_time < milliseconds:
            self.velocity_command_publisher.publish(pose2d)
            rate.sleep()


    def turn_for_milliseconds(self, angle, milliseconds):
        start_time = self.get_time_now_in_ms()
        pose2d = Pose2D()
        pose2d.x = speed
        pose2d.theta = 0

        rate = rospy.Rate(10)
        while self.get_time_now_in_ms() - start_time < milliseconds:
            self.velocity_command_publisher.publish(pose2d)
            rate.sleep()

class Receptionist:
    def __init__(self):
        self.custom_scripts_directory = os.path.dirname(sys.argv[0]) + "/../../../custom_scripts"
        self.should_abort = False
        self.is_self_driving = False

        # Publishers
        self.is_self_driving_publisher = rospy.Publisher('/robot_commander/is_self_driving', Bool, queue_size=1)
        self.is_self_driving_publisher.publish(False)

        # Subscribers
        rospy.Subscriber('/start_custom_script', Empty, self.start_custom_script, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.should_abort_callback, queue_size=1)

    def start_custom_script(self, msg):
        if not self.is_self_driving:
            self.is_self_driving = True
            self.is_self_driving_publisher.publish(self.is_self_driving)

            try:
                execfile(self.custom_scripts_directory + "/script1.py")
            except Exception as e:
                print(e)

            self.is_self_driving = False
            self.is_self_driving_publisher.publish(self.is_self_driving)

    def should_abort_callback(self, overseer_state):
        # One way to abort the thread is killing this ROS node, which automatically respawns
        if self.is_self_driving and overseer_state.data == 5:
            self.should_abort = True


if __name__ == '__main__':
    node = rospy.init_node('self_driving_executor')

    receptionist = Receptionist()

    rate = rospy.Rate(10)

    # One way to abort the thread is killing this ROS node, which automatically respawns
    while not rospy.is_shutdown():
        if receptionist.should_abort:
            break
        rate.sleep()