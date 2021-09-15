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
        self.current_path_index = 0
        self.max_path_index = 999

        # Publishers
        self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.desired_speed_publisher = rospy.Publisher('/path_follower/desired_speed', Int32, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/path_follower/current_path_index', Int32, self.current_path_index_callback)
        rospy.Subscriber('/path_follower/max_path_index', Int32, self.max_path_index_callback)

    def move_till_end_of_path(self, speed):
        self.desired_speed_publisher.publish(speed)
        self.max_path_index = 999
        rate = rospy.Rate(25)
        while (self.current_path_index != self.max_path_index):
            rate.sleep()

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

    # Subscriber Callbacks
    def current_path_index_callback(self, msg):
        self.current_path_index = msg.data

    def max_path_index_callback(self,msg):
        self.max_path_index = msg.data

class Receptionist:
    def __init__(self):
        self.script_folder = "../../../custom_script/"
        self.should_abort = False
        self.is_script_running = False
        self.previous_state = -1

        # Publishers
        self.is_script_running_publisher = rospy.Publisher('/robot_commander/is_script_running', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

        self.is_script_running_publisher.publish(False)

    def start_custom_script(self):
        try:
            execfile(self.script_folder + "script1.py")
        except Exception as e:
            print(e)

        self.is_script_running = False
        self.is_script_running_publisher.publish(False) # This will change the state in overseer.py to STOP
        
        print("completed")

    def overseer_state_callback(self, overseer_state):
        if self.previous_state == overseer_state.data:
            return
        self.previous_state = overseer_state.data

        if overseer_state.data == 2 and not self.is_script_running:
            self.is_script_running = True
            self.is_script_running_publisher.publish(True)

            custom_script_thread = threading.Thread(target=self.start_custom_script)
            custom_script_thread.setDaemon(True)
            custom_script_thread.start()
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
    
    print('robot_commander node killed. It will be respawned immediately')