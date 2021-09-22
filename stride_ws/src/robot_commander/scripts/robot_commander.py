#!/usr/bin/env python

from __future__ import division
import rospy
import time
import threading
import os
import sys
from glob import glob

from std_msgs.msg import Int32, Empty, Bool, String, Float32
from geometry_msgs.msg import Pose2D
from datetime import datetime

class RobotCommander:
    def __init__(self):
        self.current_path_index = 0
        self.max_path_index = 999

        # Publishers
        self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.desired_speed_publisher = rospy.Publisher('/robot_commander/desired_speed', Float32, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/path_follower/current_path_index', Int32, self.current_path_index_callback)
        rospy.Subscriber('/path_follower/max_path_index', Int32, self.max_path_index_callback)

    def move_till_end_of_path(self, speed):
        self.desired_speed_publisher.publish(speed)
        rate = rospy.Rate(50)
        while (self.current_path_index != self.max_path_index):
            rate.sleep()

    def move_till_index(self, speed, index):
        self.desired_speed_publisher.publish(speed)
        rate = rospy.Rate(50)
        while (self.current_path_index < index):
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
        self.filename = ""
        self.is_script_running = False
        self.previous_state = -1
        self.is_script_okay = False
        self.overseer_state = 0

        # Publishers
        self.is_script_running_publisher = rospy.Publisher('/robot_commander/is_script_running', Bool, queue_size=10)
        self.script_name_publisher = rospy.Publisher('/path_follower/script_name', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

        rospy.Subscriber('/gui/upload_script_clicked', Empty, self.upload_script_clicked_callback)

        self.is_script_running_publisher.publish(False)
        self.check_script()

    def check_script(self):

        # !!!! Need to add syntax checking !!!!
        self.is_script_okay = True
        # !!!!

        if os.path.exists(self.script_folder):
            py_files = glob(self.script_folder + '*.py')
            if len(py_files) == 1:
                filepath = py_files[0]   

                self.filename = os.path.basename(filepath)
                self.script_name_publisher.publish(self.filename)

    def start_custom_script(self):
        if self.is_script_okay:
            try:
                execfile(self.script_folder + self.filename)
            except Exception as e:
                print(e)

        self.is_script_running = False
        self.is_script_running_publisher.publish(False) # This will change the state in overseer.py to STOP
        
        print("Custom script completed execution")

    def overseer_state_callback(self, msg):
        self.overseer_state = msg.data

    def upload_script_clicked_callback(self, msg):
        self.check_script()

if __name__ == '__main__':
    node = rospy.init_node('robot_commander')

    recept = Receptionist()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # if there is a state change
        if recept.previous_state != recept.overseer_state:
            if recept.overseer_state == 2 and not recept.is_script_running:  # overseer state 2 is AUTO mode
                recept.is_script_running = True
                recept.is_script_running_publisher.publish(True)

                custom_script_thread = threading.Thread(target=recept.start_custom_script)
                custom_script_thread.setDaemon(True)
                custom_script_thread.start()
            
            # If the STOP button is clicked when the custom script is still running, kill this ROS node by breaking out of the while loop.
            # This ROS node will respawn after being killed
            elif recept.overseer_state != 2 and recept.is_script_running:
                recept.is_script_running_publisher.publish(False)
                break

            recept.previous_state = recept.overseer_state
            
        rate.sleep()