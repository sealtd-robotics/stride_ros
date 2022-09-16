#!/usr/bin/env python

import socket
import serial
import rospy

if __name__ == '__main__':
    rospy.init('rtk_forwarding')
    ser = serial.Serial("/dev/ttyTHS2")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 54000))

    while not rospy.is_shutdown():
        data, _ = s.recvfrom(1024)
        ser.write(data)

