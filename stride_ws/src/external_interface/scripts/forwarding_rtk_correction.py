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
import select

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
    s.setblocking(0)

    while not rospy.is_shutdown():
        try:
            ready = select.select([s], [], [], 0.5)
            if ready[0]:
                data = s.recv(1024)
                ser.write(data)
            else:
                print("no data, re init socket")
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("", 54008))

                group = socket.inet_aton(multicast_group)
                mreq = struct.pack('4sL', group, socket.INADDR_ANY)
                s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                s.setblocking(0)
        except Exception as e:
            print("An exception occurred")
            print(e)

# Unicast
# if __name__ == '__main__':
#     rospy.init_node('rtk_forwarding')
#     ser = serial.Serial("/dev/ttyTHS2")

#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.bind(("", 54009))

#     while not rospy.is_shutdown():
#         data, _ = s.recvfrom(1024)
#         ser.write(data)