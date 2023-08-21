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
from external_interface.msg import TargetVehicle 
from math import pi, degrees, sin, cos
import time
import zmq


def _shutdown():
    print("Node shutting down!")


def main():
    rospy.init_node('get_vehicle_input')
    rospy.on_shutdown(_shutdown)
    target_ip = "195.0.0.61"
    try:
        target_ip = rospy.get_param("target_ip")
    except:
        rospy.logerr("Target ip param is not defined. Check parameters.")
    pub = rospy.Publisher('/target', TargetVehicle, queue_size=1)

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
        msg.lateral_velocity = output_msg.lateral_velocity
        msg.roll = output_msg.roll
        msg.pitch = output_msg.pitch
        msg.acceleration_x = output_msg.acceleration_x
        msg.acceleration_y = output_msg.acceleration_y
        msg.acceleration_z = output_msg.acceleration_z
        msg.vehicle_brake = output_msg.vehicle_brake
        pub.publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Error on Vehicle Data Input")

