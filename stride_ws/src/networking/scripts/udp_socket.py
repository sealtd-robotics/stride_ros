#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from socket import socket, AF_INET, SOCK_DGRAM
import struct
import time
import rospy
from std_msgs.msg import Int32, Float32, Bool
import select
from datetime import datetime
import threading
from collections import deque

def get_time_now_in_ms():
    epoch = datetime.utcfromtimestamp(0)
    now = datetime.utcnow()
    delta = now - epoch
    return delta.total_seconds() * 1000

def monitor_heartbeat():
    while True:
        if get_time_now_in_ms() - estop_socket1_timestamp > 3000:
            estop_publisher1.publish(True)
        if get_time_now_in_ms() - estop_socket2_timestamp > 3000:
            estop_publisher2.publish(True)
        time.sleep(0.3)

if __name__ == "__main__":
    node = rospy.init_node('udp_socket')

    temp_error_word = 0

    # Publishers
    robot_temperature_publisher = rospy.Publisher('/robot_temperature', Int32, queue_size=1)
    battery_temperature_publisher = rospy.Publisher('/battery_temperature', Int32, queue_size=1)
    battery_voltage_publisher = rospy.Publisher('/battery_voltage', Float32, queue_size=1)
    estop_publisher1 = rospy.Publisher('/handheld/direct/is_estop_pressed', Bool, queue_size=1)
    estop_publisher2 = rospy.Publisher('/handheld/through_xboard/is_estop_pressed', Bool, queue_size=1)
    temp_error_word_publisher = rospy.Publisher('/temp_error_word', Int32, queue_size =1)

    # for finding moving average
    robot_temperature_history  = deque([70] * 20)
    battery_temperature_history = deque([70] * 20)

    sensors_socket = socket(AF_INET, SOCK_DGRAM)
    sensors_socket.bind(('', 54003))

    # Estop from the Arduinoo Due in the control box
    estop_socket1 = socket(AF_INET, SOCK_DGRAM)
    estop_socket1.bind(('', 54002))
    estop_socket1_timestamp = get_time_now_in_ms()

    # Estop from xboard
    estop_socket2 = socket(AF_INET, SOCK_DGRAM)
    estop_socket2.bind(('', 54004))
    estop_socket2_timestamp = get_time_now_in_ms()

    # Heartbeat timeout thread
    heartbeat_thread = threading.Thread(target=monitor_heartbeat)
    heartbeat_thread.setDaemon(True)
    heartbeat_thread.start()

    socket_list = [sensors_socket, estop_socket1, estop_socket2]
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # select.select() blocks until data arrives
        # read_sockets gives a list of sockets that have data available to be read. This prevents recvfrom() from blocking indefinitely if no data coming.
        read_sockets, write_sockets, error_sockets = select.select(socket_list, [], [])

        for sock in read_sockets:
            if sock == sensors_socket:
                dat, addr = sock.recvfrom(1024)
                robot_temperature, battery_temperature, voltage_divider = struct.unpack('3h', dat[0:6])

                # robot temperature - moving average
                robot_temperature = robot_temperature * 3.3 / 1024 * 100   # convert from digital to voltage, then to degree F
                # robot_temperature_history.pop(0)
                robot_temperature_history.popleft()
                robot_temperature_history.append(robot_temperature)
                robot_temperature_averaged = sum(robot_temperature_history) / len(robot_temperature_history)

                # battery temperature
                battery_temperature = battery_temperature * 3.3 / 1024 * 100   # convert from digital to voltage, then to degree F
                battery_temperature_history.popleft()
                battery_temperature_history.append(battery_temperature)
                battery_temperature_averaged = sum(battery_temperature_history) / len(battery_temperature_history)

                # battery_voltage
                battery_voltage = voltage_divider * 3.3 / 1024 * 8.745056384    # convert from digital to voltage of voltage divider, then to battery voltage

                # Robot and battery temperature error word
                if robot_temperature_averaged > 140 and battery_temperature_averaged > 140:
                    temp_error_word = 3
                elif robot_temperature_averaged <= 140 and battery_temperature_averaged > 140:
                    temp_error_word = 2
                elif robot_temperature_averaged > 140 and battery_temperature_averaged <= 140:
                    temp_error_word = 1
                else:
                    temp_error_word = 0

                #publishers
                robot_temperature_publisher.publish(robot_temperature_averaged)
                battery_temperature_publisher.publish(battery_temperature_averaged)
                battery_voltage_publisher.publish(battery_voltage)
                temp_error_word_publisher.publish(temp_error_word)

            elif sock == estop_socket1:
                dat, addr = sock.recvfrom(1024)
                (estop_byte,) = struct.unpack('B', dat[2])
                is_estop_pressed_1 = estop_byte & 1
                estop_publisher1.publish(is_estop_pressed_1)

                estop_socket1_timestamp = get_time_now_in_ms()
                
            elif sock == estop_socket2:
                dat, addr = sock.recvfrom(1024)
                (is_estop_pressed_2,) = struct.unpack('?', dat[0])
                estop_publisher2.publish(is_estop_pressed_2)

                estop_socket2_timestamp = get_time_now_in_ms()

        rate.sleep()