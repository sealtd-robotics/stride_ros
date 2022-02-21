#!/usr/bin/env python

# Abbreviations
# v: linear velocity
# w: angular velocity
# lat_ref: latitude reference
# long_ref: longitude reference

from __future__ import division
import rospy
import math
import os
import time
import threading
from std_msgs.msg import Int32, Float32, String, Empty, Float32MultiArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from path_follower.msg import Latlong
from shared_tools.overseer_states_constants import *

from math import cos, sin, sqrt, pi, atan2
from glob import glob


class PathFollower:
    def __init__(self):
        self.look_ahead_points = 5

        self.current_path_index = 0
        self.overseer_state = 5      # 5: STOPPED state
        self.E_factor = 0
        self.N_factor = 0
        self.lat_ref = 0
        self.long_ref = 0
        self.robot_heading = 0
        self.latitudes = []
        self.longitudes = []
        self.path_easts = []
        self.path_norths = []
        self.robot_east = 1
        self.robot_north = 1
        self.turning_radius = 999
        self.path_intervals = []
        self.stop_index = 999999
        self.max_index = 9999999

        # Publishers
        self.path_name_publisher = rospy.Publisher('/path_follower/path_name', String, queue_size=1, latch=True)
        self.path_to_follow_publisher = rospy.Publisher('/path_follower/path_to_follow', Latlong, queue_size=1, latch=True)
        self.current_path_index_publisher = rospy.Publisher('/path_follower/current_path_index', Int32, queue_size=1, latch=True)
        self.max_index_publisher = rospy.Publisher('/path_follower/max_path_index', Int32, queue_size=1, latch=True)
        self.path_intervals_publisher = rospy.Publisher('/path_follower/path_intervals', Float32MultiArray, queue_size=1, latch=True)
        self.turning_radius_publisher = rospy.Publisher('/path_follower/turning_radius', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.callback_1)
        rospy.Subscriber('/gui/upload_path_clicked', Empty, self.callback_2)
        rospy.Subscriber('/robot_velocity_command', Pose2D, self.callback_3)
        rospy.Subscriber('/robot_commander/stop_index', Int32, self.callback_4)
        rospy.Subscriber('/robot_commander/index_to_be_set', Int32, self.callback_6, queue_size=1)

        # GPS Subscribers
        rospy.Subscriber('/an_device/NavSatFix', NavSatFix, self.gps_callback_1, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_callback_2, queue_size=1)

        # Load path at startup
        self.load_path()
    
    def assign_reference_point(self, latitude, longitude):
        self.lat_ref = latitude
        self.long_ref = longitude

        e = 0.0818191908426
        R = 6378137

        self.E_factor = cos(self.lat_ref*pi/180)*R/sqrt(1-(sin(self.lat_ref*pi/180)**2*e**2))*pi/180
        self.N_factor = (1-e**2)*R/((1-(sin(self.lat_ref*pi/180)**2*e**2))*sqrt(1-(sin(self.lat_ref*pi/180)**2*e**2)))*pi/180
    
    # Latitude Longitude to North East
    def LL2NE(self, latitude, longitude):
        pos_east = (longitude - self.long_ref) * self.E_factor
        pos_north = (latitude - self.lat_ref) * self.N_factor
        return (pos_north, pos_east)

    def load_path(self):
        folder = '../../../path/'
            
        if not os.path.exists(folder):
            return

        txt_files = glob(folder + '*.txt')
        
        if len(txt_files) == 0:
            return
        
        filepath = txt_files[0]

        line_number = 1
        with open(filepath, 'r') as f:
            self.latitudes = []
            self.longitudes = []
            self.path_easts = []
            self.path_norths = []
            self.path_intervals = []

            for line in f.readlines():
                # omit the first line in file
                if line_number == 1:
                    line_number += 1
                    continue
                
                lat_long = line.split()
                latitude = float(lat_long[0])
                longitude = float(lat_long[1])

                # make the first point a reference point
                if line_number == 2:
                    self.assign_reference_point(latitude, longitude)
                    line_number += 1

                self.latitudes.append(latitude)
                self.longitudes.append(longitude)

                (pos_north, pos_east) = self.LL2NE(latitude, longitude)

                self.path_easts.append(pos_east)
                self.path_norths.append(pos_north)
        
        filename = os.path.basename(filepath)
        self.path_name_publisher.publish(filename)

        latlong = Latlong()
        latlong.latitudes = self.latitudes
        latlong.longitudes = self.longitudes
        self.path_to_follow_publisher.publish(latlong)

        self.max_index = len(self.path_easts) - 1
        self.max_index_publisher.publish(self.max_index)

        # path_intervals[3] gives the distance between index 3 and 4
        for i in range(0, len(self.path_easts) - 1):
            dx = self.path_easts[i+1] - self.path_easts[i]
            dy = self.path_norths[i+1] - self.path_norths[i]
            distance = sqrt(dx**2 + dy**2)
            self.path_intervals.append(distance)
        msg = Float32MultiArray()
        msg.data = self.path_intervals
        self.path_intervals_publisher.publish(msg)

    def update_current_path_index(self):
        if self.current_path_index == self.max_index:
            return

        # Find slope of line (m) connecting current index and the next index
        x_cur = self.path_easts[self.current_path_index]
        y_cur = self.path_norths[self.current_path_index]

        x_next = self.path_easts[min(self.current_path_index + 1, self.max_index)]
        y_next = self.path_norths[min(self.current_path_index + 1, self.max_index)]
        m = (y_next - y_cur) / (x_next - x_cur)

        # Slope of the perpendicular line to the original line
        m_perp = -1/m

        # Notes: The point-slope form of the perpendicular line at point (x_next, y_next) is as follows:
        # 0 = (y_cur - y_next) - m_perp * (x_cur - x_next)

        # Determine which side of the line the current index point lies on, indicated by the sign of k_cur
        k_cur = (y_cur - y_next) - m_perp * (x_cur - x_next)

        # Determine which side of the line the robot lies on, indicated by the sign of k_robot
        k_robot = (self.robot_north - y_next) - m_perp * (self.robot_east - x_next)

        # If the two above points are on different sides of the line
        if k_cur * k_robot < 0:
            self.current_path_index += 1
        
        self.current_path_index_publisher.publish(self.current_path_index)

    def update_turning_radius(self):
        # Notes: Since angular velocity is positive for anti-clockwise, turning raidus is positive when it's on the robot's left side
                
        # robot point
        x1 = self.robot_east
        y1 = self.robot_north

        # look-ahead point
        x2 = self.path_easts[min(self.max_index, self.stop_index, self.current_path_index + self.look_ahead_points)]
        y2 = self.path_norths[min(self.max_index, self.stop_index, self.current_path_index + self.look_ahead_points)]

        # make heading begin on the x-axis (east axis) and go counter-clockwise as positive
        adjusted_heading = pi/2 - self.robot_heading

        # distance from robot to look-ahead point
        distance = sqrt( (x2-x1)**2 + (y2-y1)**2 )

        # angle from robot to look-ahead point
        look_ahead_angle = atan2(y2-y1, x2-x1)

        # only update turning radius when distance is large enough
        # needed to prevent sudden steering when arriving at the last path index or the stop index
        if distance > 0.75:
            # angle from robot to look-ahead point
            look_ahead_angle = atan2(y2-y1, x2-x1)

            self.turning_radius = distance / (2 * sin(look_ahead_angle - adjusted_heading))

        # print(isNearMaxIndex, self.current_path_index, self.max_index, distance)
        # print('r: ', self.turning_radius, 'd: ', distance)

    # Subscriber callbacks
    def callback_1(self, msg):
        self.overseer_state = msg.data

    def callback_2(self, msg):
        self.load_path()

    def callback_3(self, msg):
        # changing look_ahead_points based on commanded speed
        # self.look_ahead_points = int(max(10, 4 * msg.x))
        self.look_ahead_points = int(max(3, 1.5 * msg.x))
        # self.look_ahead_points = 16

    def callback_4(self, msg):
        self.stop_index = msg.data

    def callback_6(self, msg):
        self.current_path_index = msg.data

    # GPS subscriber callbacks
    def gps_callback_1(self, msg):
        (self.robot_north, self.robot_east)  = self.LL2NE(msg.latitude, msg.longitude)

    def gps_callback_2(self, msg):
        self.robot_heading = msg.data    # radian

    # def gps_callback_3(self, msg):
    #     self.linear_speed_measured = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
    #     self.yaw_velocity_measured = msg.angular.z

if __name__ ==  '__main__':
    node = rospy.init_node('path_follower')

    pf = PathFollower()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if pf.overseer_state == AUTO:
            pf.update_current_path_index()
            pf.update_turning_radius()
            pf.turning_radius_publisher.publish(pf.turning_radius)
        else:
            pf.current_path_index = 0
            pf.current_path_index_publisher.publish(0)
            pf.turning_radius = 999

        rate.sleep() 
