#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

# Abbreviations:
# v: linear velocity
# w: angular velocity
# r: turning raidus of robot's geometric center

from __future__ import division
import math
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32, Int32
from joystick.msg import Stick
from shared_tools.overseer_states_constants import *

class Joystick:
    def __init__(self):
        self.v_max = rospy.get_param('v_max')
        self.w_max_spin_in_place = rospy.get_param('w_max_spin_in_place') # max angular velocity when spinning in place

        self.overseer_state = 0

        self.velocity_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.turning_radius_publisher = rospy.Publisher('/robot_turning_radius', Float32, queue_size=1)

        rospy.Subscriber('/joystick', Stick, self.publish_velocity, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

        self.rate = rospy.Rate(1)

    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data

    def publish_velocity(self, stick):
        # if not in manual, don't publish
        if not self.overseer_state == MANUAL:
            return

        v, w, r = self.joystick_to_velocities(stick.travel, stick.angle)

        # JSON, used in networking, does not allow infinity 
        if math.isinf(r):
            r = 0

        velocity = Pose2D()
        velocity.x = v
        velocity.theta = w
        
        self.velocity_publisher.publish(velocity)
        self.turning_radius_publisher.publish(r)
    
    def joystick_to_velocities(self, travel, angle):
        # returns linear_velocity, angular_velocity, turning_radius

        # travel <= 1. Scaled to slow down robot at the beginning of joystick movement
        scaled_travel = travel ** 1.5
        
        if angle == 0:
            return self.v_max * scaled_travel, 0, float('inf')
        elif abs(angle - math.pi) < 1e-5:
            return -self.v_max * scaled_travel, 0, float('inf')

        scaling = math.cos(angle)**2 * 4
        if scaling > 1:
            scaling = 1
        v_max_at_angle = self.v_max * scaling
        

        r = math.tan(-angle - math.pi / 2) * 2

        # copysign() to apply the sign of r to the first argument
        v = math.copysign( v_max_at_angle * scaled_travel, r)

        # a small angular velocity is induced when joystick is near +/- 90 degrees to make robot spin in place
        w_spin_in_place = self.w_max_spin_in_place * math.sin(angle) ** 50 * travel

        w = self.special_divide(v, r) + w_spin_in_place

        if not (angle > 0  and angle < math.pi):
            w = -w
            v = -v

        return v, w, abs(r)

    def special_divide(self, numerator, denominator):
        if denominator == 0:
            return 0
        else:
            return numerator / denominator
        

        
if __name__ ==  '__main__':
    node = rospy.init_node('joystick')

    joystick = Joystick()

    rospy.spin()