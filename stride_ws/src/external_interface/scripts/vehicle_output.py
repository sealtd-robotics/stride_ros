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
from oxford_gps_decoder.msg import OxfordIMU, StatusGPS, VelocityGPS
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import pi, degrees, sin, cos
import threading
import time
import zmq


def velocity_level(v_east, v_north, heading_rad):
	"""
	Output velocity forward and lateral.

	Parameters:
		v_east - velocity east (m/s)
		v_north - velocity north (m/s)
		heading_rad - heading (rad)
	Return:
		v_forward, v_lateral
	"""

	v_forward = v_north*cos(heading_rad) + v_east*sin(heading_rad)
	v_lateral = -v_north*sin(heading_rad) + v_east*cos(heading_rad)

	# v_forward = v_east * cos(heading_rad) + v_north * sin(heading_rad)
	# v_lateral = -v_east * sin(heading_rad) + v_north * cos(heading_rad)

	return v_forward, v_lateral


class VehicleDataSet():
	def __init__(self):
		self.longitude = 0
		self.latitude = 0
		self.heading = 0
		self.velocity = 0
		self.gps_correction = 0
		self.gps_ready = False
		self.no_of_satellites = 0


class VehicleDataOutput():
	def __init__(self):
		rospy.init_node('vehicle_output')
		self._ip = rospy.get_param("self_ip")
		output_msg = PubMsg()
		self.data = VehicleDataSet()
		self.mutex = threading.Lock()
		self.gps_time_msecs = rospy.Time.now().to_nsec() * 1e-6
		self.last_get_time = rospy.Time.now()

		ctx = zmq.Context()
		s = ctx.socket(zmq.PUB)
		s.bind("tcp://%s:50008" % self._ip)

		# OxTS ROS1 driver for ethernet
		# https://bitbucket.org/DataspeedInc/oxford_gps_eth/src/master/
		rospy.Subscriber('gps/time_ref', TimeReference, self.time_reference_cb)
		rospy.Subscriber('gps/fix', NavSatFix, self.global_pos_cb)
		rospy.Subscriber('gps/vel', TwistWithCovarianceStamped, self.velocity_cb)
		rospy.Subscriber('imu/data', Imu, self.imu_cb)
		rospy.Subscriber('gps/pos_type', String, self.correction_type_cb)
		rospy.Subscriber('gps/nav_status', String, self.gps_nav_status_cb)
		
		# CAN Bus
		rospy.Subscriber('gps/utc_time', TimeReference, self.time_reference_cb)
		rospy.Subscriber('gps/global_position', NavSatFix, self.global_pos_cb)
		rospy.Subscriber('gps/velocity', VelocityGPS, self.gps_vel_cb)
		rospy.Subscriber('gps/imu', OxfordIMU, self.gps_imu_cb)
		rospy.Subscriber('gps/status', StatusGPS, self.gps_status_cb)

		rate = rospy.Rate(100)

		while not rospy.is_shutdown():
			output_msg.Clear()
			self.current_time = self.gps_time_msecs + int(1e-6 * (rospy.Time.now() - self.last_get_time).to_nsec())
			if (self.current_time - self.gps_time_msecs) > 250:
				output_msg.gps_ready = False
				output_msg.no_of_satellites = 0
			else:
				output_msg.heading_deg = degrees(self.data.heading)
				output_msg.longitude_deg = self.data.longitude
				output_msg.latitude_deg = self.data.latitude
				output_msg.gps_correction = self.data.gps_correction
				output_msg.velocity_mps = self.data.velocity
				output_msg.gps_ready = self.data.gps_ready
				output_msg.no_of_satellites = self.data.no_of_satellites

			s.send(output_msg.SerializeToString())
			rate.sleep()

	def time_reference_cb(self, msg):
		# print("Time thread ", threading.current_thread())
		self.gps_time_msecs = int(msg.time_ref.to_nsec() * 1e-6)
		self.last_get_time = rospy.Time.now()

	def global_pos_cb(self, msg):
		# print("Pos thread ", threading.current_thread())
		self.data.longitude = msg.longitude
		self.data.latitude = msg.latitude

	def velocity_cb(self, msg):
		""" y is north, and x is east"""
		vel_north = msg.twist.twist.linear.y
		vel_east = msg.twist.twist.linear.x
		self.data.velocity, _ = velocity_level(vel_east, vel_north, self.data.heading)


	def imu_cb(self, msg):
		ori = msg.orientation
		roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
		# Convert angle from ENU to NED
		# print(degrees(yaw))
		self.data.heading = pi/2 - yaw
		# self.data.heading = yaw

		if self.data.heading < 0:
			self.data.heading += 2*pi

	def correction_type_cb(self, msg):
		if msg.data == "RTK_INTEGER":
			self.data.gps_correction = PubMsg.GpsCorrection.DGPS_RTK_INTERGER
		elif msg.data == "RTK_FLOAT":
			self.data.gps_correction = PubMsg.GpsCorrection.DGPS_RTK_FLOAT
		else:
			self.data.gps_correction = PubMsg.GpsCorrection.GPS_DIFFERENTIAL

	def gps_nav_status_cb(self, msg):
		if msg.data == "READY":
			self.data.gps_ready = True
		else:
			self.data.gps_ready = False

	def gps_vel_cb(self, msg):
		self.data.velocity = msg.forward

	def gps_imu_cb(self, msg):
		self.data.heading = msg.orientation.z

	def gps_status_cb(self, msg):
		self.data.gps_ready = msg.gps_ready
		self.data.no_of_satellites = msg.no_of_satellites
		self.data.gps_correction = msg.gps_status


if __name__ == '__main__':
	try:
		VehicleDataOutput()
	except rospy.ROSInterruptException:
		rospy.logerr("Error on Vehicle Data Output")

