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
from shared_tools.overseer_states_constants import *
from shared_tools.libcal import Compensation, Compensation_Errors
from external_interface.msg import TargetVehicle 
from path_follower.msg import Latlong
from math import pi, degrees, sin, cos
import time
import zmq
import socket
from struct import unpack

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
        self.dtc = Compensation_Errors().INVALID

        try:
            target_ip = rospy.get_param("target_ip")
        except:
            rospy.logerr("Target ip param is not defined. Check parameters.")
        udp_com = rospy.get_param("udp", True)
        udp_port = rospy.get_param("udp_port", 50008)

        pub = rospy.Publisher('/target', TargetVehicle, queue_size=1)
        rospy.Subscriber('/robot_commander/collision_point', Latlong, self.compensation_state_cb, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_cb, queue_size=1)
        rospy.Subscriber('/path_follower/vehicle_path_to_follow', Latlong, self.path_cb, queue_size=1)
        output_msg = PubMsg()

        if udp_com:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("", udp_port))
        else:
            ctx = zmq.Context()
            s = ctx.socket(zmq.SUB)
            s.setsockopt(zmq.CONFLATE, 1)
            s.connect("tcp://%s:50008" % target_ip)
            s.setsockopt(zmq.SUBSCRIBE, b'')

        while not rospy.is_shutdown():
            if udp_com:
                dat = s.recv(1024)
                data_in_decimal = unpack(str(len(dat)) + "B", dat)
                crc = (sum(data_in_decimal) - data_in_decimal[-1]) & 0xFF
                if crc != data_in_decimal[-1]:
                    print("**Failed to parse data from vehicle. Invalid data CRC {} and {}".format(crc, dat[-1]))
                    continue
                dat = dat[0:(len(dat) - 1)]
            else:
                dat = s.recv()

            try:
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
                if self.is_compensation_on:
                    self.current_index = self.comp.find_current_index(msg.latitude, msg.longitude, self.current_index)
                    self.dtc = self.comp.dist_to_collision(msg.latitude, msg.longitude, self.current_index)
                msg.distance_to_collision = self.dtc
                msg.vehicle_utc_time = output_msg.vehicle_utc_time
                pub.publish(msg)
            except:
                print("**Failed to parse data from vehicle")

    def compensation_state_cb(self,msg):
        self.is_compensation_on = False
        collision_lat = msg.latitudes[0]
        collision_long = msg.longitudes[0]
        self.comp = Compensation(self.path_to_follow)
        self.dtc = Compensation_Errors().INVALID
        self.current_index = 0
        if self.comp.pre_collision_calc(collision_lat, collision_long):
            self.is_compensation_on = True
        
    def overseer_state_cb(self,msg):
        if msg.data != self.overseer_state:
            self.is_compensation_on = False
            self.dtc = Compensation_Errors().INVALID
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

