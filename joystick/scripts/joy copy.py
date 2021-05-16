#!/usr/bin/env python

# Abbreviations:
# v: linear velocity
# w: angular velocity
# r: turning raidus of robot's geometric center

from __future__ import division
import math
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from joystick.msg import Stick

class Joystick:
    def __init__(self):
        self.v_max = rospy.get_param('~v_max')
        self.w_max_spin_in_place = rospy.get_param('~w_max_spin_in_place') # max angular velocity when spinning in place

        self.v = 0
        self.w = 0
        self.r = 0

        self.velocity_publisher = rospy.Publisher('/robot_velocity_commmand', Pose2D, queue_size=1)
        self.turning_radius_publisher = rospy.Publisher('/robot_turning_radius', Float32, queue_size=1)

        rospy.Subscriber('/joystick', Stick, self.joystick_to_velocities, queue_size=1)
    
    def joystick_to_velocities(self, stick):
        # returns linear_velocity, angular_velocity, turning_radius

        travel = stick.travel
        angle = stick.angle
        
        if angle == 0:
            return self.v_max * travel, 0, float('inf')
        elif angle == math.pi:
            return -self.v_max * travel, 0, float('inf')

        v_max_at_angle = self.v_max * math.cos(angle)**2

        r = math.tan(-angle - math.pi / 2)

        v = math.copysign( travel * v_max_at_angle, r) # applying the sign of r

        # a small angular velocity is induced when joystick is near +/- 90 degrees to make robot spin in place
        w_spin_in_place = self.w_max_spin_in_place * math.sin(angle) ** 100 * travel

        w = self.special_divide(v, r) + w_spin_in_place

        if not (angle > 0  and angle < math.pi):
            w = -w
            v = -v

        self.v = v
        self.w = w
        self.r = abs(r)

    def special_divide(self, numerator, denominator):
        if denominator == 0:
            return 0
        else:
            return numerator / denominator
        

        
if __name__ ==  '__main__':
    node = rospy.init_node('joystick')

    joystick = Joystick()

    velocity = Pose2D()
    velocity.x = joystick.v
    velocity.theta = joystick.w

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        rate.sleep()