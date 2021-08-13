#!/usr/bin/env python

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, UInt16, UInt8, Int32, Bool, Empty
from geometry_msgs.msg import Pose2D, Vector3, Twist
from sensor_msgs.msg import NavSatFix, Imu
from joystick.msg import Stick
import time
import threading
from datetime import datetime

from autobahn.twisted.websocket import WebSocketServerProtocol
from autobahn.twisted.websocket import WebSocketServerFactory
from twisted.internet import task
import json
import sys
from twisted.python import log
from twisted.internet import reactor

class RosInterface:
    def __init__(self):
        self.gps_callback_sleep_time = 0.1

        # javascript naming convention
        self.robotState = {
            "type": "robotState",
            "robotVelocityCommand": {
                "v": 0,
                "w": 0
            },
            "robotTurningRadius": 0,
            "overseerState": 0,
            "doesBrakeWhenStopped": False,
            "gps": {
                "status": -1,
                "latitude": 0,
                "longitude": 0,
                "latitudeVariance": 0,
                "longitudeVariance": 0,
                "northVelocity": 0,
                "eastVelocity": 0,
                "zAngularVelocity": 0,
                "xAcceleration": 0,
                "yAcceleration": 0,
                "zAcceleration": 0,
                "systemStatus": 0,
                "filterStatus": 0,
                "heading": 0,
                "xMagnetometer": 0,
                "yMagnetometer": 0,
                "zMagnetometer": 0,
                "magneticCalibrationStatus": 0,
                "magneticCalibrationProgress": 0,
                "magneticCalibrationError": 0,
            }
        }
        
        # Subscribers
        rospy.Subscriber('/robot_velocity_command', Pose2D, self.subscriber_callback_1, queue_size=1)
        rospy.Subscriber('/robot_turning_radius', Float32, self.subscriber_callback_2, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_3, queue_size=1)
        rospy.Subscriber('/motor_controller_network/does_brake_when_stopped', Bool, self.subscriber_callback_4, queue_size=1)

        rospy.Subscriber('/an_device/NavSatFix', NavSatFix, self.gps_subscriber_callback_1, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/Twist', Twist, self.gps_subscriber_callback_2, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/Imu', Imu, self.gps_subscriber_callback_3, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/system_status', UInt16, self.gps_subscriber_callback_4, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/filter_status', UInt16, self.gps_subscriber_callback_5, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/heading', Float32, self.gps_subscriber_callback_6, queue_size=1) # time.sleep() in callback for throttling, used with queue_size=1
        rospy.Subscriber('/an_device/magnetometers', Vector3, self.gps_subscriber_callback_7, queue_size=1)
        rospy.Subscriber('/an_device/magnetic_calibration/status', UInt8, self.gps_subscriber_callback_8, queue_size=1)
        rospy.Subscriber('/an_device/magnetic_calibration/progress', UInt8, self.gps_subscriber_callback_9, queue_size=1)
        rospy.Subscriber('/an_device/magnetic_calibration/error', UInt8, self.gps_subscriber_callback_10, queue_size=1)

        # Publishers
        self.joystick_publisher = rospy.Publisher('/joystick', Stick, queue_size=1)
        self.stop_clicked_publisher = rospy.Publisher('/gui/stop_clicked', Empty, queue_size=1)
        self.enable_manual_publisher = rospy.Publisher('/gui/enable_manual_clicked', Empty, queue_size=1)
        self.heartbeat_publisher = rospy.Publisher('/gui/heartbeat', Empty, queue_size=1)
        self.toggle_brake_publisher = rospy.Publisher('/gui/brake_when_stopped_toggled', Empty, queue_size=1)
        self.start_calibration_publisher = rospy.Publisher('/an_device/magnetic_calibration/calibrate', UInt8, queue_size=1)
        
    def subscriber_callback_1(self, msg):
        self.robotState['robotVelocityCommand']['v'] = msg.x
        self.robotState['robotVelocityCommand']['w'] = msg.theta

    def subscriber_callback_2(self, msg):
        self.robotState['robotTurningRadius'] = msg.data

    def subscriber_callback_3(self, msg):
        self.robotState['overseerState'] = msg.data

    def subscriber_callback_4(self, msg):
        self.robotState['doesBrakeWhenStopped'] = msg.data

    def gps_subscriber_callback_1(self, msg):
        self.robotState['gps']['status'] = msg.status.status
        self.robotState['gps']['latitude'] = msg.latitude
        self.robotState['gps']['longtitude'] = msg.longtitude
        self.robotState['gps']['latitudeVariance'] = msg.position_covariance[0]
        self.robotState['gps']['longtitudeVariance'] = msg.position_covariance[4]
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_2(self, msg):
        self.robotState['gps']['northVelocity'] = msg.linear.x
        self.robotState['gps']['eastVelocity'] = msg.linear.y
        self.robotState['gps']['zAngularVelocity'] = msg.angular.z
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_3(self, msg):
        self.robotState['gps']['xAcceleration'] = msg.linear_acceleration.x
        self.robotState['gps']['yAcceleration'] = msg.linear_acceleration.y
        self.robotState['gps']['zAcceleration'] = msg.linear_acceleration.z
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_4(self, msg):
        self.robotState['gps']['systemStatus'] = msg.data
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_5(self, msg):
        self.robotState['gps']['filterStatus'] = msg.data
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_6(self, msg):
        self.robotState['gps']['heading'] = msg.data
        time.sleep(self.gps_callback_sleep_time) # for throttling high publishing rate

    def gps_subscriber_callback_7(self, msg):
        self.robotState['gps']['xMagnetometer'] = msg.x
        self.robotState['gps']['yMagnetometer'] = msg.y
        self.robotState['gps']['zMagnetometer'] = msg.z

    def gps_subscriber_callback_8(self, msg):
        self.robotState['gps']['magneticCalibrationStatus'] = msg.data

    def gps_subscriber_callback_9(self, msg):
        self.robotState['gps']['magneticCalibrationProgress'] = msg.data

    def gps_subscriber_callback_10(self, msg):
        self.robotState['gps']['magneticCalibrationError'] = msg.data


class MyServerProtocol(WebSocketServerProtocol):
    websocket_client_count = 0

    def __init__(self):
        super(MyServerProtocol, self).__init__()
        self.is_closed = False
        self.ros_interface = RosInterface()
    
    def onConnect(self, request):
        print("Client connecting: {}".format(request.peer))
        print("Number of clients: {}".format(MyServerProtocol.websocket_client_count))

    def onOpen(self):
        print("WebSocket connection open.")
        MyServerProtocol.websocket_client_count += 1

        self.thread1 = threading.Thread(target=self.transmit_robot_state)
        self.thread1.setDaemon(True)
        self.thread1.start()

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {} bytes".format(len(payload)))
        else:
            print("Text message received: {}".format(payload.decode('utf8')))

        # echo back message verbatim
        self.sendMessage(payload, isBinary)

    def onClose(self, wasClean, code, reason):
        self.is_closed = True
        MyServerProtocol.websocket_client_count -= 1
        print("WebSocket connection closed: {}".format(reason))



    def transmit_robot_state(self):
        rate = rospy.Rate(10)
        while not self.is_closed:
            newRobotState = self.ros_interface.robotState
            newRobotState["websocketClientCount"] = MyServerProtocol.websocket_client_count

            robotStateMessage = json.dumps(newRobotState, ensure_ascii = False).encode('utf8')
            reactor.callFromThread(self.sendMessage, robotStateMessage, False)
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