#!/usr/bin/env python

from __future__ import division
import rospy
import time
import threading
import os
import sys
import math
from glob import glob

from std_msgs.msg import Int32, Empty, Bool, String, Float32, Float32MultiArray
from geometry_msgs.msg import Pose2D, Twist
from datetime import datetime

class RobotCommander:
    def __init__(self):
        self.current_path_index = -1
        self.max_path_index = -1
        self.path_intervals = []
        self.robot_speed = -1
        self.robot_heading = -1

        # Publishers
        self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.desired_speed_publisher = rospy.Publisher('/robot_commander/desired_speed', Float32, queue_size=1)
        self.stop_index_publisher = rospy.Publisher('/robot_commander/stop_index', Int32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/path_follower/current_path_index', Int32, self.current_path_index_callback)
        rospy.Subscriber('/path_follower/max_path_index', Int32, self.max_path_index_callback)
        rospy.Subscriber('/path_follower/path_intervals', Float32MultiArray, self.path_intervals_callback)
        rospy.Subscriber('/an_device/Twist', Twist, self.gps_twist_callback, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_heading_callback, queue_size=1)

        # blocking until all attributes have been updated by subscriber callbacks
        while (self.current_path_index == -1 or self.max_path_index == -1 or
                self.path_intervals == [] or self.robot_speed == -1 or self.robot_heading == -1):
            time.sleep(0.1)


    def move_till_end_of_path(self, speed):
        self.desired_speed_publisher.publish(speed)
        rate = rospy.Rate(50)
        while (self.current_path_index < self.max_path_index):
            rate.sleep()

    def move_till_index(self, speed, index):
        self.desired_speed_publisher.publish(speed)
        rate = rospy.Rate(50)
        while (self.current_path_index < index):
            rate.sleep()

    # maybe add a try-except statement to catch zero angular veloctiy and zero tolerance
    def rotate_till_heading(self, angular_velocity, heading, heading_tolerance = 3):
        pose2d = Pose2D()
        pose2d.x = 0
        pose2d.theta = angular_velocity
        self.velocity_command_publisher.publish(pose2d)
        
        heading = heading % 360

        heading_radian = heading / math.pi * 180
        tolerance_radian = heading_tolerance / math.pi * 180

        lower_bound = (heading_radian - tolerance_radian) % 360
        upper_bound = (heading_radian + tolerance_radian) % 360

        rate = rospy.Rate(50)
        if upper_bound > lower_bound:
            while self.robot_heading < lower_bound or self.robot_heading > upper_bound:
                rate.sleep()
        else:
            while self.robot_heading > lower_bound and self.robot_heading < upper_bound:
                rate.sleep()
        
        pose2d.x = 0
        pose2d.theta = 0
        self.velocity_command_publisher.publish(pose2d)

    def decel_to_stop_at_index(self, stop_index):
        self.stop_index_publisher.publish(stop_index)

        frequency = 50
        rate = rospy.Rate(frequency)
        period = 1/frequency

        # kinemetic equation: vf^2 = vi^2 + 2*a*d
        vi = self.robot_speed
        d = sum(self.path_intervals[ self.current_path_index : stop_index ])
        a = -vi**2 / 2 / d

        while (self.current_path_index < stop_index):
            vi = vi + a*period # a is negative
            vi = max(vi, 0.3) # prevent zero velocity before reaching the stop_index
            self.desired_speed_publisher.publish(vi)

            rate.sleep()

        self.desired_speed_publisher.publish(0)
        self.stop_index_publisher.publish(999999)

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

    def max_path_index_callback(self, msg):
        self.max_path_index = msg.data

    def path_intervals_callback(self, msg):
        self.path_intervals = msg.data

    def gps_twist_callback(self, msg):
        self.robot_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)

    def gps_heading_callback(self, msg):
        self.robot_heading = msg.data # in radian

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