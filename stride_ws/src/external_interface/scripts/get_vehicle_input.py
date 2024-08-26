#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from __future__ import print_function
import os 
import sys

script_dir = os.path.dirname(__file__)
proto_dir = os.path.join(script_dir, '..')
sys.path.append(proto_dir)

from proto_src import PubMsg
import rospy
from sensor_msgs.msg import NavSatFix, TimeReference, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool, Int32
from geographic_msgs.msg import GeoPoint
from shared_tools.overseer_states_constants import *
from shared_tools.libcal import Compensation
from external_interface.msg import TargetVehicle 
from path_follower.msg import Latlong
from math import pi, degrees, sin, cos
import time
import zmq

is_compensation_on = False
overseer_state = 0

def _shutdown():
    print("Node shutting down!")

class TargetVehicleInput(object):
    def __init__(self):
        rospy.init_node('get_vehicle_input')
        rospy.on_shutdown(_shutdown)
        target_ip = "195.0.0.61"
        self.is_compensation_on = False
        self.comp = None
        self.current_index = 0
        self.path_to_follow = {}
        self.overseer_state = IDLE

        try:
            target_ip = rospy.get_param("target_ip")
        except:
            rospy.logerr("Target ip param is not defined. Check parameters.")

        pub = rospy.Publisher('/target', TargetVehicle, queue_size=1)
        rospy.Subscriber('/robot_commander/collision_point', GeoPoint, self.compensation_state_cb, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_cb, queue_size=1)
        rospy.Subscriber('/path_follower/vehicle_path_to_follow', Latlong, self.path_cb, queue_size=1)
        output_msg = PubMsg()

        ctx = zmq.Context()
        s = ctx.socket(zmq.SUB)
        s.connect("tcp://%s:50008" % target_ip)
        s.setsockopt(zmq.SUBSCRIBE, b'')

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            dat = s.recv()
            output_msg.Clear()
            output_msg.ParseFromString(dat)
            msg = TargetVehicle()
            msg.heading = output_msg.heading_deg
            msg.longitude = output_msg.longitude_deg
            msg.latitude = output_msg.latitude_deg
            msg.velocity = output_msg.velocity_mps
            msg.gps_correction_type = output_msg.gps_correction + 1
            msg.gps_ready = output_msg.gps_ready
            msg.no_of_satellites = output_msg.no_of_satellites
            # msg.lateral_velocity = output_msg.lateral_velocity
            # msg.roll = output_msg.roll
            # msg.pitch = output_msg.pitch
            # msg.acceleration_x = output_msg.acceleration_x
            # msg.acceleration_y = output_msg.acceleration_y
            # msg.acceleration_z = output_msg.acceleration_z
            # msg.vehicle_brake = output_msg.vehicle_brake
            if self.is_compensation_on:
                self.current_index = self.comp.find_current_index(msg.latitude, msg.longitude, self.current_index)
                self.dtc = self.comp.dist_to_collision(msg.latitude, msg.longitude, self.current_index)
            msg.distance_to_collision = self.dtc
            pub.publish(msg)
            rate.sleep()

    def compensation_state_cb(self,msg):
        self.is_compensation_on = False
        collision_lat = msg.latitude
        collision_long = msg.longitude
        self.comp = Compensation(self.path_to_follow)
        self.dtc = -1
        if self.comp.pre_collision_calc(collision_lat, collision_long):
            self.is_compensation_on = True
        
    def overseer_state_cb(self,msg):
        if msg.data != self.overseer_state:
            self.is_compensation_on = False
            self.dtc = -1
            self.overseer_state = msg.data

    def path_cb(self, msg):
        self.path_to_follow = {}
        self.path_to_follow['latitudes'] = msg.latitudes
        self.path_to_follow['longitudes'] = msg.longitudes

if __name__ == '__main__':
    try:
        TargetVehicleInput()
    except rospy.ROSInterruptException:
        rospy.logerr("Error on Vehicle Data Input")

