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
        self.script_folder = "../../../custom_script/"
        self.should_abort = False
        self.is_script_running = False

        # Publishers
        self.is_script_running_publisher = rospy.Publisher('/robot_commander/is_script_running', Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber('/start_custom_script', Empty, self.start_custom_script, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback, queue_size=10)

        self.is_script_running_publisher.publish(False)

    def start_custom_script(self):
        try:
            execfile(self.script_folder + "script1.py")
        except Exception as e:
            print(e)

        # The following 3 lines must run in this specific sequence
        self.is_script_running_publisher.publish(False) # This will change the state in overseer.py to STOP
        time.sleep(0.5)
        self.is_script_running = False
        

    def overseer_state_callback(self, overseer_state):
        if overseer_state.data == 2 and not self.is_script_running:
            self.is_script_running = True
            self.is_script_running_publisher.publish(True)
            
            self.start_custom_script()

        elif overseer_state.data != 2 and self.is_script_running:
            self.should_abort = True

if __name__ == '__main__':
    node = rospy.init_node('robot_commander')

    receptionist = Receptionist()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # One way to abort a thread is killing this ROS node, which automatically respawns
        if receptionist.should_abort:
            receptionist.is_script_running_publisher.publish(False) # if node is about to be killed, signal False
            break
        rate.sleep()