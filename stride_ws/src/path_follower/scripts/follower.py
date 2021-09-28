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
from std_msgs.msg import Int32, Float32, String, Empty, Float32MultiArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from path_follower.msg import Latlong

from math import cos, sin, sqrt, pi, atan2
from glob import glob


class PathFollower:
    def __init__(self):
        self.look_ahead_points = 6

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
        self.turning_radius = 1
        self.desired_speed = 0
        self.path_intervals = []
        self.stop_index = 999999

        # Publishers
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.path_name_publisher = rospy.Publisher('/path_follower/path_name', String, queue_size=1, latch=True)
        self.path_to_follow_publisher = rospy.Publisher('/path_follower/path_to_follow', Latlong, queue_size=1, latch=True)
        self.current_path_index_publisher = rospy.Publisher('/path_follower/current_path_index', Int32, queue_size=1, latch=True)
        self.max_index_publisher = rospy.Publisher('/path_follower/max_path_index', Int32, queue_size=1, latch=True)
        self.path_intervals_publisher = rospy.Publisher('/path_follower/path_intervals', Float32MultiArray, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_1)
        rospy.Subscriber('/gui/upload_path_clicked', Empty, self.subscriber_callback_2)
        rospy.Subscriber('/robot_commander/desired_speed', Float32, self.subscriber_callback_3)
        rospy.Subscriber('/robot_commander/stop_index', Int32, self.subscriber_callback_4)

        # GPS Subscribers
        rospy.Subscriber('/an_device/NavSatFix', NavSatFix, self.gps_subscriber_callback_1, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_subscriber_callback_2, queue_size=1)

        # Load path at startup
        self.load_path()

        # rospy.Subscriber('/an_device/Twist', Twist, self.gps_subscriber_callback_3, queue_size=1)
    
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

        self.max_index_publisher.publish(len(self.path_easts) - 1)

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
        max_index = len(self.path_easts) - 1

        if self.current_path_index == max_index:
            return

        # Find slope of line (m) connecting current index and the next index
        x_cur = self.path_easts[self.current_path_index]
        y_cur = self.path_norths[self.current_path_index]

        x_next = self.path_easts[min(self.current_path_index + 1, max_index)]
        y_next = self.path_norths[min(self.current_path_index + 1, max_index)]
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
        
        max_index = len(self.path_easts) - 1
        
        # robot point
        x1 = self.robot_east
        y1 = self.robot_north

        # look-ahead point
        x2 = self.path_easts[min(max_index, self.stop_index, self.current_path_index + self.look_ahead_points)]
        y2 = self.path_norths[min(max_index, self.stop_index, self.current_path_index + self.look_ahead_points)]

        # make heading begin on the x-axis (east axis) and go counter-clockwise as positive
        adjusted_heading = pi/2 - self.robot_heading

        # distance from robot to look-ahead point
        distance = sqrt( (x2-x1)**2 + (y2-y1)**2 )

        # angle from robot to look-ahead point
        look_ahead_angle = atan2(y2-y1, x2-x1)

        self.turning_radius = distance / (2 * sin(look_ahead_angle - adjusted_heading))

        # prevent sudden rotation when arriving at the last path index and the stop index
        if (self.current_path_index >= max_index - 1 or self.current_path_index >= self.stop_index - 1) and distance < 0.1:
            self.turning_radius = 99999

    def publish_robot_velocity(self):
        max_index = len(self.path_easts) - 1

        if self.current_path_index == max_index:
            pose2d = Pose2D()
            pose2d.x = 0
            pose2d.theta = 0
            self.robot_velocity_publisher.publish(pose2d)
            return
        pose2d = Pose2D()
        pose2d.x = self.desired_speed
        pose2d.theta = self.desired_speed / self.turning_radius

        self.robot_velocity_publisher.publish(pose2d)

    # Subscriber callbacks
    def subscriber_callback_1(self, msg):
        self.overseer_state = msg.data

    def subscriber_callback_2(self, msg):
        self.load_path()

    def subscriber_callback_3(self, msg):
        self.desired_speed = msg.data

    def subscriber_callback_4(self, msg):
        self.stop_index = msg.data

    # GPS subscriber callbacks
    def gps_subscriber_callback_1(self, msg):
        (self.robot_north, self.robot_east)  = self.LL2NE(msg.latitude, msg.longitude)

    def gps_subscriber_callback_2(self, msg):
        self.robot_heading = msg.data    # radian

    # def gps_subscriber_callback_3(self, msg):
    #     self.linear_speed_measured = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
    #     self.yaw_velocity_measured = msg.angular.z

if __name__ ==  '__main__':
    node = rospy.init_node('path_follower')

    pf = PathFollower()

    should_reset_variables = False
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if pf.overseer_state == 2:    # 2 is the autonomous (AUTO) state
            pf.update_current_path_index()
            pf.update_turning_radius()
            pf.publish_robot_velocity()
            should_reset_variables = True

        elif should_reset_variables:
            pf.current_path_index = 0
            pf.current_path_index_publisher.publish(0)
            pf.desired_speed = 0

            should_reset_variables = False

        rate.sleep() 
