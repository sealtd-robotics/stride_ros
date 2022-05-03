#!/usr/bin/env python

from __future__ import division
import queue
import rospy
import numpy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Pose2D, Vector3
from nav_msgs.msg import Odometry
import time
from shared_tools.utils import find_rate_limited_speed
from shared_tools.overseer_states_constants import *
from tf.transformations import euler_from_quaternion

class Decender:
    def __init__(self):
        self.overseer_state = STOPPED
        self.pitch = 0

        # Publishers
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.callback_1)
        rospy.Subscriber('/an_device/pitch', Float32, self.callback_2)
        rospy.Subscriber('/gps/euler_orientation', Vector3, self.callback_3, queue_size=1)

    def callback_1(self, msg):
        self.overseer_state = msg.data

    def callback_2(self, msg):
        self.pitch = msg.data
        time.sleep(0.1)

    def callback_3(self, msg):
        self.pitch = msg.y
        time.sleep(0.1)

if __name__ == '__main__':
    node = rospy.init_node('decender')
    d = Decender()

    speed_goal = 0.5
    speed_rate = 0.1
    initial_speed = 0

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        state = d.overseer_state

        if state == DESCENDING:
            # On entering this state
            if state != previous_state:
                initial_time = time.time()

            limited_speed = find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)

            # pitch is negative when robot tilts forward
            pose2d = Pose2D()
            pose2d.x = limited_speed * numpy.sign(d.pitch) * -1
            d.robot_velocity_publisher.publish(pose2d)

        previous_state = state
        rate.sleep()