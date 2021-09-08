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
from std_msgs.msg import Int32, Float32, String, Empty
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from math import cos, sin, sqrt, pi, atan2
from glob import glob

class PathFollower:
    def __init__(self):
        self.point_spacing = 0.15   # meters. Is this needed?
        self.look_ahead_points = 4

        self.current_path_index = 0
        self.overseer_state = 5      # 5: STOPPED state
        self.E_factor = 0
        self.N_factor = 0
        self.lat_ref = 0
        self.long_ref = 0
        self.robot_heading = 0
        self.path_easts = []
        self.path_norths = []
        self.robot_east = 1
        self.robot_north = 1
        self.turning_radius = 1

        # Publishers
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.path_name_publisher = rospy.Publisher('/path_follower/path_name', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_1, queue_size=10)
        rospy.Subscriber('/gui/upload_path_clicked', Empty, self.subscriber_callback_2, queue_size=1)

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
        txt_files = glob('../../../path/*.txt')
        
        if len(txt_files) == 0:
            return
        
        filepath = txt_files[0]
        

        self.path_easts = []
        self.path_norths = []

        line_number = 1
        with open(filepath, 'r') as f:
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

                (pos_north, pos_east) = self.LL2NE(latitude, longitude)

                self.path_easts.append(pos_east)
                self.path_norths.append(pos_north)
        
        filename = os.path.basename(filepath)
        self.path_name_publisher.publish(filename)

        # for i in range(0, len(self.path_easts)):
        #     print(self.path_easts[i], self.path_norths[i])

    def update_current_path_index(self):
        # if current index is the last index, return
        if self.current_path_index == len(self.path_easts) - 1:
            return

        # Find slope of line (m) connecting current index and the next index
        x_cur = self.path_easts[self.current_path_index]
        y_cur = self.path_norths[self.current_path_index]

        x_next = self.path_easts[self.current_path_index + 1]
        y_next = self.path_norths[self.current_path_index + 1]
        m = (y_next - y_cur) / (x_next - x_cur)

        # Slope of the perpendicular line to the original line
        m_perp = -1/m

        # Notes: The point-slope form of the perpendicular line at point (x_next, y_next) is as follows:
        # 0 = m_perp * (x - x_next) / (y - y_next)

        # Determine which side of the line the current index point lies on, indicated by the sign of k_cur
        k_cur = m_perp * (x_cur - x_next) / (y_cur - y_next)

        # Determine which side of the line the robot lies on, indicated by the sign of k_robot
        k_robot = m_perp * (self.robot_east - x_next) / (self.robot_north - y_next)

        # If the two above points are on different sides of the line
        if k_cur * k_robot < 0:
            self.current_path_index += 1

    def update_turning_radius(self):
        # Since angular velocity is positive for anti-clockwise, turning raidus is positive when it's on the robot's left side
        
        # robot point
        x1 = self.robot_east
        y1 = self.robot_north
        # print('x1', x1)
        # print('y1', y1)

        # look-ahead point
        x2 = self.path_easts[self.current_path_index + self.look_ahead_points]
        y2 = self.path_norths[self.current_path_index + self.look_ahead_points]
        # print('x2', x2)
        # print('y2', y2)

        # make heading begin on the x-axis and go counter-clockwise as positive
        adjusted_heading = -self.robot_heading - pi/2

        # distance from robot to look-ahead point
        distance = sqrt( (x2-x1)**2 + (y2-y1)**2 )

        # angle from robot to look-ahead point
        look_ahead_angle = atan2(y2-y1, x2-x1)

        self.turning_radius = distance / (2 * sin(look_ahead_angle - adjusted_heading))
        # print(distance)
        # print(look_ahead_angle)
        # print(adjusted_heading)
        # print(self.turning_radius)

    def publish_robot_velocity(self):

        pose2d = Pose2D()
        pose2d.x = 0.5
        pose2d.theta = 0.5 / self.turning_radius

        self.robot_velocity_publisher.publish(pose2d)



    # Subscriber callbacks
    def subscriber_callback_1(self, msg):
        self.overseer_state = msg.data

    def subscriber_callback_2(self, msg):
        self.load_path()


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

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if pf.overseer_state == 2:    # 2 is the autonomous (AUTO) state
            pf.update_current_path_index()
            pf.update_turning_radius()
            pf.publish_robot_velocity()
        else:
            pf.current_path_index = 0
        rate.sleep() 
