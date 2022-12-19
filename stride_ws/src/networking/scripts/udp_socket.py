#!/usr/bin/env python

from socket import socket, AF_INET, SOCK_DGRAM
import struct
import time
import rospy
from std_msgs.msg import Int32, Float32, Bool
import select
from datetime import datetime
import threading
from collections import deque
import struct

brake_command = False

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

def monitor_portenta_heartbeat():
    while True:
        if get_time_now_in_ms() - portenta_socket_timestamp > 2000:
            portenta_heartbeat_publisher.publish(False)
        time.sleep(0.2)

def brake_command_callback(msg):
    global brake_command
    brake_command = msg.data
    brake_output = struct.pack('<?',brake_command)
    brake_socket2.sendto(brake_output, ('195.0.0.231',54006))

if __name__ == "__main__":
    node = rospy.init_node('udp_socket')

    #Brake subsriber
    rospy.Subscriber('/brake_command', Bool, brake_command_callback, queue_size=1)

    # Publishers
    robot_temperature_publisher = rospy.Publisher('/robot_temperature', Int32, queue_size=1)
    battery_temperature_publisher = rospy.Publisher('/battery_temperature', Int32, queue_size=1)
    battery_voltage_publisher = rospy.Publisher('/battery_voltage', Float32, queue_size=1)
    estop_publisher1 = rospy.Publisher('/handheld/direct/is_estop_pressed', Bool, queue_size=1)
    estop_publisher2 = rospy.Publisher('/handheld/through_xboard/is_estop_pressed', Bool, queue_size=1)
    brake_status_publisher = rospy.Publisher('/brake_status', Int32, queue_size = 1)
    fullyseated_L_publisher = rospy.Publisher('/fullyseated_L', Int32, queue_size = 1)
    fullyseated_R_publisher = rospy.Publisher('/fullyseated_R', Int32, queue_size = 1)
    portenta_heartbeat_publisher = rospy.Publisher('/portenta_heartbeat', Bool, queue_size = 1)

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

    # Brake: Arduino to jetson
    brake_socket = socket(AF_INET, SOCK_DGRAM)
    brake_socket.bind(('',54005)) 

    # Brake: Jetson to arduino
    brake_socket2 = socket(AF_INET, SOCK_DGRAM) 

    #Portenta heartbeat
    portenta_socket = socket(AF_INET, SOCK_DGRAM)
    portenta_socket.bind(('',54007)) 
    portenta_socket_timestamp = get_time_now_in_ms()

    # Heartbeat timeout thread
    heartbeat_thread = threading.Thread(target=monitor_heartbeat)
    heartbeat_thread.setDaemon(True)
    heartbeat_thread.start()

    # Portenta Heartbeat timeout thread
    portenta_heartbeat_thread = threading.Thread(target=monitor_portenta_heartbeat)
    portenta_heartbeat_thread.setDaemon(True)
    portenta_heartbeat_thread.start()

    socket_list = [sensors_socket, estop_socket1, estop_socket2, brake_socket, portenta_socket]
    rate = rospy.Rate(50)
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

                robot_temperature_publisher.publish(robot_temperature_averaged)
                battery_temperature_publisher.publish(battery_temperature_averaged)
                battery_voltage_publisher.publish(battery_voltage)

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

            elif sock == brake_socket:
                dat, addr = sock.recvfrom(1024) 
                brake_status, fullyseated_L, fullyseated_R = struct.unpack('3B',dat[0:3]) 
                
                #publish vars from arduino
                brake_status_publisher.publish(brake_status)   
                fullyseated_L_publisher.publish(fullyseated_L)    
                fullyseated_R_publisher.publish(fullyseated_R) 

            elif sock == portenta_socket:
                dat, addr = sock.recvfrom(1024)
                (is_portenta_ok,) = struct.unpack('?', dat[0])
                portenta_heartbeat_publisher.publish(is_portenta_ok) #Publish arduino heartbeat state

                portenta_socket_timestamp = get_time_now_in_ms()

        rate.sleep()