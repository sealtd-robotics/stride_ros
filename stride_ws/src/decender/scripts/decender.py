#!/usr/bin/env python

from __future__ import division
import rospy
import numpy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Pose2D


class Decender:
    def __init__(self):
        self.overseer_state = 5 # 5: Stopped state
        self.pitch = 0

        # Publishers
        self.robot_velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.callback_1)
        rospy.Subscriber('/an_device/pitch', Float32, self.callback_2)

    def callback_1(self, msg):
        self.overseer_state = msg.data

    def callback_2(self, msg):
        self.pitch = msg.data

if __name__ == '__main__':
    node = rospy.init_node('decender')
    d = Decender()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if d.overseer_state == 6: # 6: Decending state
            # pitch is negative when robot tilts forward
            pose2d = Pose2D()
            pose2d.x = 0.25 * numpy.sign(d.pitch) * -1
            d.robot_velocity_publisher.publish(pose2d)

        rate.sleep()