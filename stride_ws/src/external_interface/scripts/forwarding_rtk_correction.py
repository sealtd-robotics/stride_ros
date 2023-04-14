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

if __name__ == '__main__':
    rospy.init_node('rtk_forwarding')
    ser = serial.Serial("/dev/ttyTHS2")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 54000))

    while not rospy.is_shutdown():
        data, _ = s.recvfrom(1024)
        ser.write(data)

