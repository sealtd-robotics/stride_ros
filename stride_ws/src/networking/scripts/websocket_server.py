#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

# abbreviations
# mc: motor controller

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, UInt16, UInt8, Int16, Int32, Bool, Empty, String
from external_interface.msg import TargetVehicle
from geometry_msgs.msg import Pose2D, Vector3, Twist, TwistWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, Imu
from joystick.msg import Stick
from path_follower.msg import Latlong
from sbg_driver.msg import SbgEkfNav, SbgGpsPos, SbgEkfEuler
import time
import threading
from datetime import datetime
import os
from glob import glob

from autobahn.twisted.websocket import WebSocketServerProtocol
from autobahn.twisted.websocket import WebSocketServerFactory
from twisted.internet import task
import json
import sys
from twisted.python import log
from twisted.internet import reactor


class RosInterface:
    def __init__(self, websocket):
        self.gps_callback_sleep_time = 0.1
        self.websocket = websocket
        self.gnss_pos_status = 0

        # Does not get transmitted continuously
        self.path_to_follow = {
            "latitudes": [],
            "longitudes": []
        }

        # robotState:
        #   -javascript naming convention used
        #   -additional keys will be added before sending out
        #   -gets transmited continuously at certain rate
        self.robotState = {
            "type": "robotState",
            "robotVelocityCommand": {
                "v": 0,
                "w": 0
            },
            "robotTurningRadius": 0,
            "overseerState": 0,
            # "websocketClientCount" will be added before sending out
            "doesBrakeWhenStopped": False,
            "robotTemperature": 0,
            "batteryTemperature": 0,
            "batteryVoltage": 0,
            "tempErrorWord": 0,
            "motorControllers": {
                "leftFront": {
                    "state": 0,
                    "heartbeatNmt": 0,
                    "motorCurrentDraw": 0,
                    "wheelRpmActual": 0,
                    "errorWord": 0,
                    "isHeartbeatTimeout": False,
                    "windingTemperature": 0,
                },
                "leftBack": {
                    "state": 0,
                    "heartbeatNmt": 0,
                    "motorCurrentDraw": 0,
                    "wheelRpmActual": 0,
                    "errorWord": 0,
                    "isHeartbeatTimeout": False,
                    "windingTemperature": 0,
                },
                "rightFront": {
                    "state": 0,
                    "heartbeatNmt": 0,
                    "motorCurrentDraw": 0,
                    "wheelRpmActual": 0,
                    "errorWord": 0,
                    "isHeartbeatTimeout": False,
                    "windingTemperature": 0,
                },
                "rightBack": {
                    "state": 0,
                    "heartbeatNmt": 0,
                    "motorCurrentDraw": 0,
                    "wheelRpmActual": 0,
                    "errorWord": 0,
                    "isHeartbeatTimeout": False,
                    "windingTemperature": 0,
                },
            },
            "gps": {
                "status": -1,
                "latitude": 0,
                "longitude": 0,
                "northVelocity": 0,
                "eastVelocity": 0,
                "zAngularVelocity": 0,
                "heading": 0,
            },
            "pathFollower": {
                "pathName": "",
                "scriptName": "",
            },
            "targetVehicle": {
                "latitude": 0,
                "longitude": 0,
                "heading": 0,
                "velocity": 0,
                "gps_ready": False,
                "gps_correction_type": 0,
            }
        }
        
        # Subscribers
        rospy.Subscriber('/robot_velocity_command', Pose2D, self.subscriber_callback_1, queue_size=1)
        rospy.Subscriber('/robot_turning_radius', Float32, self.subscriber_callback_2, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller_network/does_brake_when_stopped', Bool, self.subscriber_callback_4, queue_size=1)
        rospy.Subscriber('/robot_temperature', Int32, self.subscriber_callback_5, queue_size=1)
        rospy.Subscriber('/battery_temperature', Int32, self.subscriber_callback_6, queue_size=1)
        rospy.Subscriber('/battery_voltage', Float32, self.subscriber_callback_7, queue_size=1)
        rospy.Subscriber('/temp_error_word', Int32, self.temp_error_word_callback, queue_size=1)
        rospy.Subscriber('/target', TargetVehicle, self.subscriber_callback_8, queue_size=1)
        rospy.Subscriber('/csv_converted', Empty, self.subscriber_callback_9, queue_size=1)
        rospy.Subscriber('/robot_commander/command_message', String, self.subscriber_callback_10, queue_size=20)
        rospy.Subscriber('/overseer/error_message', String, self.subscriber_callback_11, queue_size=20)

        # GPS Subcribers
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_subscriber_callback_1, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/gps/vel', TwistWithCovarianceStamped, self.gps_subscriber_callback_3, queue_size=1)
        rospy.Subscriber('/gps/euler_orientation', Vector3, self.gps_subscriber_callback_11, queue_size=1)
        rospy.Subscriber('/gps/pos_type', String, self.gps_oxford_pos_type_callback, queue_size=1)

        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.gps_sbg_pos_callback, queue_size=1)
        rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, self.gps_sbg_gnss_pos_callback, queue_size=1)
        rospy.Subscriber('/imu/velocity', TwistStamped, self.gps_sbg_vel_callback, queue_size=1)
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.gps_sbg_imu_callback, queue_size=1)
        

        # Motor Controller Subscribers
        # Left Front
        rospy.Subscriber('/motor_controller/left_front/state', Int32, self.left_front_mc_callback_1, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/heartbeat_nmt', Int32, self.left_front_mc_callback_2, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/motor_current_draw', Float32, self.left_front_mc_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/wheel_rpm_actual', Float32, self.left_front_mc_callback_4, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/error_word', Int32, self.left_front_mc_callback_5, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/is_heartbeat_timeout', Bool, self.left_front_mc_callback_6, queue_size=1)
        rospy.Subscriber('/motor_controller/left_front/winding_temperature', Int32, self.left_front_mc_callback_7, queue_size=1)

        # Left Back
        rospy.Subscriber('/motor_controller/left_back/state', Int32, self.left_back_mc_callback_1, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/heartbeat_nmt', Int32, self.left_back_mc_callback_2, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/motor_current_draw', Float32, self.left_back_mc_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/wheel_rpm_actual', Float32, self.left_back_mc_callback_4, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/error_word', Int32, self.left_back_mc_callback_5, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/is_heartbeat_timeout', Bool, self.left_back_mc_callback_6, queue_size=1)
        rospy.Subscriber('/motor_controller/left_back/winding_temperature', Int32, self.left_back_mc_callback_7, queue_size=1)

        # Right Front
        rospy.Subscriber('/motor_controller/right_front/state', Int32, self.right_front_mc_callback_1, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/heartbeat_nmt', Int32, self.right_front_mc_callback_2, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/motor_current_draw', Float32, self.right_front_mc_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/wheel_rpm_actual', Float32, self.right_front_mc_callback_4, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/error_word', Int32, self.right_front_mc_callback_5, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/is_heartbeat_timeout', Bool, self.right_front_mc_callback_6, queue_size=1)
        rospy.Subscriber('/motor_controller/right_front/winding_temperature', Int32, self.right_front_mc_callback_7, queue_size=1)

        # Right Back
        rospy.Subscriber('/motor_controller/right_back/state', Int32, self.right_back_mc_callback_1, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/heartbeat_nmt', Int32, self.right_back_mc_callback_2, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/motor_current_draw', Float32, self.right_back_mc_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/wheel_rpm_actual', Float32, self.right_back_mc_callback_4, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/error_word', Int32, self.right_back_mc_callback_5, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/is_heartbeat_timeout', Bool, self.right_back_mc_callback_6, queue_size=1)
        rospy.Subscriber('/motor_controller/right_back/winding_temperature', Int32, self.right_back_mc_callback_7, queue_size=1)

        # Path Follower Subscribers
        rospy.Subscriber('/path_follower/path_name', String, self.path_follower_callback_1, queue_size=1)
        rospy.Subscriber('/path_follower/path_to_follow', Latlong, self.path_follower_callback_2, queue_size=1)
        rospy.Subscriber('/path_follower/script_name', String, self.path_follower_callback_3, queue_size=1)

        # Publishers
        self.joystick_publisher = rospy.Publisher('/joystick', Stick, queue_size=1)
        self.stop_clicked_publisher = rospy.Publisher('/gui/stop_clicked', Empty, queue_size=1)
        self.enable_manual_publisher = rospy.Publisher('/gui/enable_manual_clicked', Empty, queue_size=1)
        self.idle_clicked_publisher = rospy.Publisher('/gui/idle_clicked', Empty, queue_size=1)
        self.heartbeat_publisher = rospy.Publisher('/gui/heartbeat', Empty, queue_size=1)
        self.toggle_brake_publisher = rospy.Publisher('/gui/brake_when_stopped_toggled', Empty, queue_size=1)
        self.start_calibration_publisher = rospy.Publisher('/an_device/magnetic_calibration/calibrate', UInt8, queue_size=1)
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.start_following_publisher = rospy.Publisher('/gui/start_path_following_clicked', Empty, queue_size=1)
        self.upload_path_publisher = rospy.Publisher('/gui/upload_path_clicked', Empty, queue_size=1)
        self.upload_script_publisher = rospy.Publisher('/gui/upload_script_clicked', Empty, queue_size=1)
        self.return_to_start_publisher = rospy.Publisher('/gui/return_to_start_clicked', Empty, queue_size=1)
        
    # Callbacks
    def subscriber_callback_1(self, msg):
        self.robotState['robotVelocityCommand']['v'] = round(msg.x, 3)
        self.robotState['robotVelocityCommand']['w'] = round(msg.theta, 3)

    def subscriber_callback_2(self, msg):
        self.robotState['robotTurningRadius'] = round(msg.data, 2)

    def subscriber_callback_3(self, msg):
        self.robotState['overseerState'] = msg.data

    def subscriber_callback_4(self, msg):
        self.robotState['doesBrakeWhenStopped'] = msg.data

    def subscriber_callback_5(self, msg):
        self.robotState['robotTemperature'] = msg.data

    def subscriber_callback_6(self, msg):
        self.robotState['batteryTemperature'] = msg.data

    def subscriber_callback_7(self, msg):
        self.robotState['batteryVoltage'] = round(msg.data, 2)

    def temp_error_word_callback(self, msg):
        self.robotState['tempErrorWord'] = msg.data

    def subscriber_callback_8(self, msg):
        self.robotState['targetVehicle']['latitude'] = msg.latitude
        self.robotState['targetVehicle']['longitude'] = msg.longitude
        self.robotState['targetVehicle']['heading'] = round(msg.heading, 1)
        self.robotState['targetVehicle']['velocity'] = round(msg.velocity, 3)
        self.robotState['targetVehicle']['gps_ready'] = msg.gps_ready
        self.robotState['targetVehicle']['gps_correction_type'] = msg.gps_correction_type

    def subscriber_callback_9(self, msg):
        csvCreatedMsg = json.dumps({'type': 'createdCSV',}, ensure_ascii = False).encode('utf8')
        reactor.callFromThread(self.websocket.sendMessage, csvCreatedMsg, False)

    def subscriber_callback_10(self, msg):
        command_message = json.dumps({'type': 'commandMessage', 'commandMessage': msg.data}, ensure_ascii = False).encode('utf8')
        reactor.callFromThread(self.websocket.sendMessage, command_message, False)

    def subscriber_callback_11(self, msg):
        error_message = json.dumps({'type': 'errorMessage', 'errorMessage': msg.data}, ensure_ascii = False).encode('utf8')
        reactor.callFromThread(self.websocket.sendMessage, error_message, False)

    # Motor Controller Callbacks
    # Left Front
    def left_front_mc_callback_1(self, msg):
        self.robotState['motorControllers']['leftFront']['state'] = msg.data
    def left_front_mc_callback_2(self, msg):
        self.robotState['motorControllers']['leftFront']['heartbeatNmt'] = msg.data
    def left_front_mc_callback_3(self, msg):
        self.robotState['motorControllers']['leftFront']['motorCurrentDraw'] = round(msg.data, 3)
    def left_front_mc_callback_4(self, msg):
        self.robotState['motorControllers']['leftFront']['wheelRpmActual'] = round(msg.data, 2)
    def left_front_mc_callback_5(self, msg):
        self.robotState['motorControllers']['leftFront']['errorWord'] = msg.data
    def left_front_mc_callback_6(self, msg):
        self.robotState['motorControllers']['leftFront']['isHeartbeatTimeout'] = msg.data
    def left_front_mc_callback_7(self, msg):
        self.robotState['motorControllers']['leftFront']['windingTemperature'] = msg.data

    # Left Back
    def left_back_mc_callback_1(self, msg):
        self.robotState['motorControllers']['leftBack']['state'] = msg.data
    def left_back_mc_callback_2(self, msg):
        self.robotState['motorControllers']['leftBack']['heartbeatNmt'] = msg.data
    def left_back_mc_callback_3(self, msg):
        self.robotState['motorControllers']['leftBack']['motorCurrentDraw'] = round(msg.data ,3)
    def left_back_mc_callback_4(self, msg):
        self.robotState['motorControllers']['leftBack']['wheelRpmActual'] = round(msg.data, 2)
    def left_back_mc_callback_5(self, msg):
        self.robotState['motorControllers']['leftBack']['errorWord'] = msg.data
    def left_back_mc_callback_6(self, msg):
        self.robotState['motorControllers']['leftBack']['isHeartbeatTimeout'] = msg.data
    def left_back_mc_callback_7(self, msg):
        self.robotState['motorControllers']['leftBack']['windingTemperature'] = msg.data

    # Right Front
    def right_front_mc_callback_1(self, msg):
        self.robotState['motorControllers']['rightFront']['state'] = msg.data
    def right_front_mc_callback_2(self, msg):
        self.robotState['motorControllers']['rightFront']['heartbeatNmt'] = msg.data
    def right_front_mc_callback_3(self, msg):
        self.robotState['motorControllers']['rightFront']['motorCurrentDraw'] = round(msg.data, 3)
    def right_front_mc_callback_4(self, msg):
        self.robotState['motorControllers']['rightFront']['wheelRpmActual'] = round(msg.data, 2)
    def right_front_mc_callback_5(self, msg):
        self.robotState['motorControllers']['rightFront']['errorWord'] = msg.data
    def right_front_mc_callback_6(self, msg):
        self.robotState['motorControllers']['rightFront']['isHeartbeatTimeout'] = msg.data
    def right_front_mc_callback_7(self, msg):
        self.robotState['motorControllers']['rightFront']['windingTemperature'] = msg.data

    # Right Back
    def right_back_mc_callback_1(self, msg):
        self.robotState['motorControllers']['rightBack']['state'] = msg.data
    def right_back_mc_callback_2(self, msg):
        self.robotState['motorControllers']['rightBack']['heartbeatNmt'] = msg.data
    def right_back_mc_callback_3(self, msg):
        self.robotState['motorControllers']['rightBack']['motorCurrentDraw'] = round(msg.data, 3)
    def right_back_mc_callback_4(self, msg):
        self.robotState['motorControllers']['rightBack']['wheelRpmActual'] = round(msg.data, 2)
    def right_back_mc_callback_5(self, msg):
        self.robotState['motorControllers']['rightBack']['errorWord'] = msg.data
    def right_back_mc_callback_6(self, msg):
        self.robotState['motorControllers']['rightBack']['isHeartbeatTimeout'] = msg.data
    def right_back_mc_callback_7(self, msg):
        self.robotState['motorControllers']['rightBack']['windingTemperature'] = msg.data

    # GPS callbacks
    def gps_subscriber_callback_1(self, msg):
        # self.robotState['gps']['status'] = msg.status.status
        self.robotState['gps']['status'] = self.gnss_pos_status # temp patch
        self.robotState['gps']['latitude'] = msg.latitude
        self.robotState['gps']['longitude'] = msg.longitude
        time.sleep(self.gps_callback_sleep_time) # prevent frequenty update from high publishing rate

    def gps_subscriber_callback_3(self, msg):
        self.robotState['gps']['northVelocity'] = round(msg.twist.twist.linear.y, 3)
        self.robotState['gps']['eastVelocity'] = round(msg.twist.twist.linear.x, 3)
        self.robotState['gps']['zAngularVelocity'] = round(msg.twist.twist.angular.z, 3)
        time.sleep(self.gps_callback_sleep_time) # prevent frequenty update from high publishing rate

    def gps_subscriber_callback_11(self, msg):
        self.robotState['gps']['heading'] = round(msg.z, 3)
        time.sleep(self.gps_callback_sleep_time)

    def gps_oxford_pos_type_callback(self, msg):
        pos_type = msg.data
        if (pos_type == "RTK_INTEGER"):
            self.gnss_pos_status = 7
        elif (pos_type == "RTK_FLOAT"):
            self.gnss_pos_status = 6
        elif (pos_type == "DIFF_PSEUDORANGE"):
            self.gnss_pos_status = 4
        else:
            self.gnss_pos_status = 3
        time.sleep(self.gps_callback_sleep_time)

    ### SBG GPS ###
    def gps_sbg_pos_callback(self, msg):
        self.robotState['gps']['latitude'] = msg.latitude
        self.robotState['gps']['longitude'] = msg.longitude
        time.sleep(self.gps_callback_sleep_time) # prevent frequenty update from high publishing rate

    def gps_sbg_gnss_pos_callback(self, msg):
        self.robotState['gps']['status'] = msg.status.type
        time.sleep(self.gps_callback_sleep_time)

    def gps_sbg_vel_callback(self, msg):
        self.robotState['gps']['northVelocity'] = round(msg.twist.linear.x, 3)
        self.robotState['gps']['eastVelocity'] = round(msg.twist.linear.y, 3)
        self.robotState['gps']['zAngularVelocity'] = msg.twist.angular.z
        time.sleep(self.gps_callback_sleep_time)

    def gps_sbg_imu_callback(self, msg):
        self.robotState['gps']['heading'] = round(msg.angle.z, 3)
        time.sleep(self.gps_callback_sleep_time)

    # Path follower callbacks
    def path_follower_callback_1(self, msg):
        self.robotState['pathFollower']['pathName'] = msg.data

    def path_follower_callback_2(self, msg):
        self.path_to_follow['latitudes'] = msg.latitudes
        self.path_to_follow['longitudes'] = msg.longitudes

    def path_follower_callback_3(self, msg):
        self.robotState['pathFollower']['scriptName'] = msg.data


class MyServerProtocol(WebSocketServerProtocol):
    # All connections will share the class variables
    websocket_client_count = 0
    ros_interface = 0
    shared_path = {'type': '', 'latitudes':[], 'longitudes':[]}

    def __init__(self):
        MyServerProtocol.ros_interface = RosInterface(self)
    # Side note: The init function of this class is called before each connection and is only relevent to that connection

    # Each connection will have its own onConnect, onOpen, onMessage, and onClose 
    def onConnect(self, request):
        print("Client connecting: {}".format(request.peer))

    def onOpen(self):
        MyServerProtocol.websocket_client_count += 1
        print("WebSocket connection open.")
        print("Number of clients: {}".format(MyServerProtocol.websocket_client_count))

        self.is_connected = True
        self.message_arrival_time = self.get_time_now_in_ms()

        self.thread1 = threading.Thread(target=self.transmit_robot_state)
        self.thread1.setDaemon(True)
        self.thread1.start()

        self.thread2 = threading.Thread(target=self.transmit_shared_path)
        self.thread2.setDaemon(True)
        self.thread2.start()

        self.thread3 = threading.Thread(target=self.close_if_no_incoming_messages)
        self.thread3.setDaemon(True)
        self.thread3.start()

        self.thread4 = threading.Thread(target=self.transmit_path_to_follow)
        self.thread4.setDaemon(True)
        self.thread4.start()

    def onMessage(self, payload, isBinary):
        # if isBinary:
        #     print("Binary message received: {} bytes".format(len(payload)))
        # else:
        #     print("Text message received: {}".format(payload.decode('utf8')))

        # # echo back message verbatim
        # self.sendMessage(payload, isBinary)

        message = json.loads(payload.decode('utf8'))

        self.message_arrival_time = self.get_time_now_in_ms()

        if message['type'] == '/joystick':
            stick = Stick()
            stick.travel = message['travel']
            stick.angle = message['angle']
            MyServerProtocol.ros_interface.joystick_publisher.publish(stick)
        elif message['type'] == '/gui/stop_clicked':
            MyServerProtocol.ros_interface.stop_clicked_publisher.publish()
        elif message['type'] == '/gui/enable_manual_clicked':
            MyServerProtocol.ros_interface.enable_manual_publisher.publish()
        elif message['type'] == '/gui/idle_clicked':
            MyServerProtocol.ros_interface.idle_clicked_publisher.publish()
        elif message['type'] == '/gui/heartbeat':
            MyServerProtocol.ros_interface.heartbeat_publisher.publish()
        elif message['type'] == '/gui/brake_when_stopped_toggled':
            MyServerProtocol.ros_interface.toggle_brake_publisher.publish()
        elif message['type'] == '/an_device/magnetic_calibration/calibrate':
            MyServerProtocol.ros_interface.start_calibration_publisher.publish(message['method'])
        elif message['type'] == '/robot_velocity_command':
            pose2d = Pose2D()
            pose2d.x = message['x']
            pose2d.theta = message['theta']
            MyServerProtocol.ros_interface.robot_velocity_publisher.publish(pose2d)
        elif message['type'] == 'sharedPath':
            MyServerProtocol.shared_path = {'type': 'sharedPath', 'latitudes': message['latitudes'], 'longitudes': message['longitudes']}
        elif message['type'] == '/gui/start_path_following_clicked':
            MyServerProtocol.ros_interface.start_following_publisher.publish()
        elif message['type'] == '/gui/return_to_start_clicked':
            MyServerProtocol.ros_interface.return_to_start_publisher.publish()
        elif message['type'] == '/gui/upload_path_clicked':
            folder = '../../../path/'
            
            if not os.path.exists(folder):
                os.makedirs(folder)

            # remove all txt files (there should only be one txt file allowed in the folder)
            for path in glob (folder + '*.txt'):
                os.remove(path)
            
            with open(folder + message['filename'], 'w+') as f:
                f.write(message['fileContent'])
            MyServerProtocol.ros_interface.upload_path_publisher.publish()

        elif message['type'] == '/gui/upload_script_clicked':
            folder = '../../../custom_script/'
            
            if not os.path.exists(folder):
                os.makedirs(folder)

            # remove all txt files (there should only be one txt file allowed in the folder)
            for path in glob (folder + '*.py'):
                os.remove(path)
            
            with open(folder + message['filename'], 'w+') as f:
                f.write(message['fileContent'])
            MyServerProtocol.ros_interface.upload_script_publisher.publish()

    def onClose(self, wasClean, code, reason):
        self.is_connected = False
        MyServerProtocol.websocket_client_count -= 1
        print("WebSocket connection closed: {}".format(reason))
        print("Number of clients: {}".format(MyServerProtocol.websocket_client_count))

    def get_time_now_in_ms(self):
        epoch = datetime.utcfromtimestamp(0)
        now = datetime.utcnow()
        delta = now - epoch
        return delta.total_seconds() * 1000

    def transmit_robot_state(self):
        rate = rospy.Rate(10)
        while self.is_connected:
            newRobotState = MyServerProtocol.ros_interface.robotState
            newRobotState["websocketClientCount"] = MyServerProtocol.websocket_client_count

            robotStateMessage = json.dumps(newRobotState, ensure_ascii = False).encode('utf8')
            reactor.callFromThread(self.sendMessage, robotStateMessage, False)
            rate.sleep()

    def transmit_shared_path(self):
        rate = rospy.Rate(2)
        previous_path = MyServerProtocol.shared_path
        while self.is_connected:
            is_new_path = id(previous_path) != id(MyServerProtocol.shared_path)
            
            if is_new_path:
                pathMessage = json.dumps(MyServerProtocol.shared_path, ensure_ascii = False).encode('utf8')
                reactor.callFromThread(self.sendMessage, pathMessage, False)

                previous_path = MyServerProtocol.shared_path # shallow copy
                print('Path has been shared')
            
            rate.sleep()

    def close_if_no_incoming_messages(self):
        rate = rospy.Rate(2)
        while self.is_connected:
            if self.get_time_now_in_ms() - self.message_arrival_time > 4000:
                reactor.callFromThread(self.sendClose)

            rate.sleep()

    def transmit_path_to_follow(self):
        rate = rospy.Rate(2)
        previous_latitudes = []
        previous_longitudes = []
        while self.is_connected:
            is_new_latitudes = id(previous_latitudes) != id(MyServerProtocol.ros_interface.path_to_follow['latitudes'])
            is_new_longitudes = id(previous_longitudes) != id(MyServerProtocol.ros_interface.path_to_follow['longitudes'])
            
            if is_new_latitudes and is_new_longitudes:
                path_to_follow = {'type': '/path_follower/path_to_follow',
                                    'latitudes':MyServerProtocol.ros_interface.path_to_follow['latitudes'],
                                    'longitudes':MyServerProtocol.ros_interface.path_to_follow['longitudes']}

                pathMessage = json.dumps(path_to_follow, ensure_ascii = False).encode('utf8')
                reactor.callFromThread(self.sendMessage, pathMessage, False)

                previous_latitudes = MyServerProtocol.ros_interface.path_to_follow['latitudes'] # shallow copy
                previous_longitudes = MyServerProtocol.ros_interface.path_to_follow['longitudes']
            
            rate.sleep()

def shutdown():
    if rospy.is_shutdown():
        reactor.stop()

if __name__ ==  '__main__':
    node = rospy.init_node('websocket_server')

    websocket_client_count = 0

    log.startLogging(sys.stdout)
    factory = WebSocketServerFactory()
    factory.protocol = MyServerProtocol

    reactor.listenTCP(9000, factory)
    task.LoopingCall(shutdown).start(1)
    reactor.run()