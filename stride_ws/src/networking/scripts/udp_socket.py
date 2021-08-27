#!/usr/bin/env python

from socket import socket, AF_INET, SOCK_DGRAM
import struct
import time
import rospy
from std_msgs.msg import Int16, Float32

if __name__ == "__main__":
    node = rospy.init_node('udp_socket')

    # Publishers
    robot_temperature_publisher = rospy.Publisher('/robot_temperature', Int16, queue_size=1)
    battery_temperature_publisher = rospy.Publisher('/battery_temperature', Int16, queue_size=1)
    battery_voltage_publisher = rospy.Publisher('/battery_voltage', Float32, queue_size=1)

    # for finding moving average
    robot_temperature_history  = [70] * 20 

    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 54003))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dat, addr = s.recvfrom(1024)
        robot_temperature, battery_temperature, voltage_divider = struct.unpack('3h', dat[0:6])

        # robot temperature - moving average
        robot_temperature = robot_temperature * 5.0 / 1024 * 100   # convert from digital to voltage, then to degree F
        robot_temperature_history.pop(0)
        robot_temperature_history.append(robot_temperature)
        robot_temperature_averaged = sum(robot_temperature_history) / len(robot_temperature_history)

        # battery temperature
        battery_temperature = battery_temperature * 5.0 / 1024 * 100   # convert from digital to voltage, then to degree F

        # battery_voltage
        battery_voltage = voltage_divider * 5.0 / 1024 * 7.7678    # convert from digital to voltage of voltage divider, then to battery voltage

        robot_temperature_publisher.publish(robot_temperature_averaged)
        battery_temperature_publisher.publish(battery_temperature)
        battery_voltage_publisher.publish(battery_voltage)

        rate.sleep()