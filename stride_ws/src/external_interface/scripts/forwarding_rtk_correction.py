#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

import socket
import serial
import rospy
import struct

# Multicast
if __name__ == '__main__':
    multicast_group = ('234.5.6.7')
    
    rospy.init_node('rtk_forwarding')
    ser = serial.Serial("/dev/ttyTHS2")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 54008))

    group = socket.inet_aton(multicast_group)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    while not rospy.is_shutdown():
        data, address = s.recvfrom(1024)
        print(data)
        ser.write(data)

# Unicast
# if __name__ == '__main__':
#     rospy.init_node('rtk_forwarding')
#     ser = serial.Serial("/dev/ttyTHS2")

#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.bind(("", 54009))

#     while not rospy.is_shutdown():
#         data, _ = s.recvfrom(1024)
#         ser.write(data)