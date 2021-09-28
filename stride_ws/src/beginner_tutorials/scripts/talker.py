#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String, Float32, Empty
from geometry_msgs.msg import Twist
from can_interface.msg import WheelRPM
from sensor_msgs.msg import NavSatFix
import time

def talker():
    pub1 = rospy.Publisher('a1', String, queue_size=10)
    pub2 = rospy.Publisher('a2', String, queue_size=10)
    pub3 = rospy.Publisher('a3', String, queue_size=10)
    pub4 = rospy.Publisher('a4', String, queue_size=10)
    pub5 = rospy.Publisher('a5', String, queue_size=10)
    pub6 = rospy.Publisher('a6', String, queue_size=10)
    pub7 = rospy.Publisher('a7', String, queue_size=10)
    pub8 = rospy.Publisher('a8', String, queue_size=10)
    pub9 = rospy.Publisher('a9', String, queue_size=10)
    pub10 = rospy.Publisher('a10', Float32, queue_size=10)
    pub11 = rospy.Publisher('/wheel_rpm_command', WheelRPM, queue_size=10)
    pub12 = rospy.Publisher('robot_velocity_commmand', Twist, queue_size=10)
    pub13 = rospy.Publisher('/an_device/NavSatFix', NavSatFix, queue_size=10)
    pub14 = rospy.Publisher('/test', Empty, queue_size=10)
    pub15 = rospy.Publisher('/blocking', Empty, queue_size=10)

    count = 0
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str1 = "a hello world %s" % rospy.get_time()
        hello_str2 = "b hello world %s" % rospy.get_time()
        hello_str3 = "c hello world %s" % rospy.get_time()
        hello_str4 = "d hello world %s" % rospy.get_time()
        hello_str = "d hello world %s" % rospy.get_time()
        
        wheel_rpm = WheelRPM()
        wheel_rpm.left_front = -6.0
        wheel_rpm.left_back = -7.0
        wheel_rpm.right_front = -8.0
        wheel_rpm.right_back = -9.0

        robot_vel_cmd = Twist()
        robot_vel_cmd.linear.x = 0 / 9.54929658551
        robot_vel_cmd.angular.z = 0

        pub1.publish(hello_str1)
        pub2.publish(hello_str2)
        pub3.publish(hello_str3)
        pub4.publish(hello_str4)
        pub5.publish(hello_str)
        pub6.publish(hello_str)
        pub7.publish(hello_str)
        pub8.publish(hello_str)
        pub9.publish(hello_str)
        pub10.publish(float('inf'))
        pub11.publish(wheel_rpm)

        robot_vel_cmd.linear.x = 0
        robot_vel_cmd.angular.z = 0
        pub12.publish(robot_vel_cmd)

        count += 0.000001
        navSatFix = NavSatFix()
        navSatFix.latitude = count
        navSatFix.longitude = count
        pub13.publish(navSatFix)

        pub14.publish()
        pub15.publish()

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
