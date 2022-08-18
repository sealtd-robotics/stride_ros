#!/usr/bin/env python

from __future__ import division
import rospy
import time
import threading
import os
import math
import numpy as np
from glob import glob
from libcal import LL_NE, CheckBoundariesEnter

from std_msgs.msg import Int32, Empty, Bool, String, Float32, Float32MultiArray
from geometry_msgs.msg import Pose2D, Twist, TwistWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav
from external_interface.msg import TargetVehicle
from datetime import datetime
from shared_tools.utils import find_rate_limited_speed as _find_rate_limited_speed
from shared_tools.overseer_states_constants import *

# Global control robot commander across threads

let_script_runs = False
dash_line = "----------------------"

class RobotCommander:
    def __init__(self):
        self.current_path_index = 0
        self.max_path_index = -1
        self.path_intervals = []
        self.robot_speed = -1
        self.robot_heading = -1
        self.turning_radius = 999
        self.limiter_initial_speed = 0

        # Publishers
        self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
        self.stop_index_publisher = rospy.Publisher('/robot_commander/stop_index', Int32, queue_size=1)
        self.spin_velocity_publisher = rospy.Publisher('/robot_commander/spin_in_place_velocity', Float32, queue_size=1)
        self.set_index_publisher = rospy.Publisher('/robot_commander/index_to_be_set', Int32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/path_follower/current_path_index', Int32, self.current_path_index_callback)
        rospy.Subscriber('/path_follower/max_path_index', Int32, self.max_path_index_callback)
        rospy.Subscriber('/path_follower/path_intervals', Float32MultiArray, self.path_intervals_callback)
        rospy.Subscriber('/path_follower/turning_radius', Float32, self.turning_radius_callback)
        rospy.Subscriber('/an_device/Twist', Twist, self.gps_callback_1, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_callback_2, queue_size=1)
        rospy.Subscriber('/gps/euler_orientation', Vector3, self.gps_orientation_callback, queue_size=1)
        rospy.Subscriber('/gps/vel', TwistWithCovarianceStamped, self.gps_velocity_callback, queue_size=1)
        rospy.Subscriber('/target', TargetVehicle, self.target_callback, queue_size=1)

        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.gps_sbg_euler_callback, queue_size=1)
        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.gps_sbg_nav_callback, queue_size=1)

        # blocking until these attributes have been updated by subscriber callbacks
        while (self.max_path_index == -1 or self.path_intervals == [] or self.robot_speed == -1 or self.robot_heading == -1 or self.turning_radius == 999):
            time.sleep(0.1)

    def _display_message(self, message):
        print(message)
        command_message_publisher.publish(message)

    def _send_velocity_command_using_radius(self, speed):
        self.limiter_initial_speed = speed

        pose2d = Pose2D()
        pose2d.x = speed
        w = speed / self.turning_radius
        if (speed < 0):
            w *= -1.0
        pose2d.theta = w

        self.velocity_command_publisher.publish(pose2d)

    def move_until_end_of_path(self, speed_goal, speed_rate):
        if not let_script_runs:
            return
        self._display_message('Executing move_until_end_of_path')
        rate = rospy.Rate(50)

        initial_time = time.time()
        initial_speed = self.limiter_initial_speed
        while (self.current_path_index < self.max_path_index) and let_script_runs:
            limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)
            self._send_velocity_command_using_radius(limited_speed)
            rate.sleep()

    def brake_to_stop(self, speed_rate):
        if not let_script_runs:
            return
        self._display_message('Executing brake_to_stop')
        rate = rospy.Rate(50)
        speed_goal = 0

        initial_time = time.time()
        initial_speed = self.limiter_initial_speed
        while (self.robot_speed > 0.1 \
            and self.current_path_index < self.max_path_index) \
            and let_script_runs:
            limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)
            self._send_velocity_command_using_radius(limited_speed)
            rate.sleep()

    def move_until_index(self, speed_goal, speed_rate, index):
        global let_script_runs
        if not let_script_runs:
            return
        if index > self.max_path_index:
            let_script_runs = False
            print("Input index must not exceed max path index.")
            return
        self._display_message('Executing move_until_index')
        rate = rospy.Rate(50)
        initial_time = time.time()
        initial_speed = self.limiter_initial_speed
        while (self.current_path_index < index) and let_script_runs:
            limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)
            self._send_velocity_command_using_radius(limited_speed)
            rate.sleep()

    def set_index(self, index):
        if not let_script_runs:
            return
        self._display_message('Executing set_index')
        self.set_index_publisher.publish(index)
        rate = rospy.Rate(50)
        while (self.current_path_index != index) and let_script_runs:
            rate.sleep()

    # maybe add a try-except statement to catch zero angular velocity and zero tolerance
    def rotate_until_heading(self, angular_velocity, heading, heading_tolerance = 3):
        if not let_script_runs:
            return
        self._display_message('Executing rotate_until_heading')
        if heading >= 0:
            heading = heading % 360
        else:
            heading = heading % -360

        heading_radian = heading / 180 * math.pi
        tolerance_radian = heading_tolerance / 180 * math.pi

        print(heading_radian)
        print(tolerance_radian)
        if heading >=3: 
            lower_bound = (heading_radian - tolerance_radian) % (2*math.pi)
            upper_bound = (heading_radian + tolerance_radian) % (2*math.pi)
        elif heading < 3 and heading > -3:
            lower_bound = (heading_radian - tolerance_radian) 
            upper_bound = (heading_radian + tolerance_radian) 
            print(lower_bound)
            print(upper_bound)
        else:
            lower_bound = (heading_radian - tolerance_radian) % -(2*math.pi)
            upper_bound = (heading_radian + tolerance_radian) % -(2*math.pi)
            print(lower_bound)
            print(upper_bound)

        pose2d = Pose2D()
        pose2d.x = 0
        pose2d.theta = -angular_velocity

        rate = rospy.Rate(50)
        if upper_bound > lower_bound:
            while (self.robot_heading < lower_bound \
                    or self.robot_heading > upper_bound) \
                    and let_script_runs:
                self.velocity_command_publisher.publish(pose2d)
                rate.sleep()
        else:
            while (self.robot_heading > lower_bound \
                and self.robot_heading < upper_bound) \
                and let_script_runs:
                self.velocity_command_publisher.publish(pose2d)
                rate.sleep()

        pose2d.theta = 0
        self.velocity_command_publisher.publish(pose2d)   

    def decel_to_stop_at_index(self, stop_index):
        global let_script_runs
        if not let_script_runs:
            return
        if stop_index > self.max_path_index:
            let_script_runs = False
            print("Input stop index must not exceed max path index.")
            return
        self._display_message('Executing decel_to_stop_at_index')
        self.stop_index_publisher.publish(stop_index)

        frequency = 50
        rate = rospy.Rate(frequency)
        period = 1/frequency

        # kinematic equation: vf^2 = vi^2 + 2*a*d
        vi = self.robot_speed
        d = sum(self.path_intervals[ self.current_path_index : stop_index ])
        a = -vi**2 / (2*d)

        while (self.current_path_index < stop_index) and let_script_runs:
            vi = vi + a*period # a is negative
            vi = max(vi, 0.3) # prevent zero velocity before reaching the stop_index
            self._send_velocity_command_using_radius(vi)
            rate.sleep()

        while (self.robot_speed > 0.1) and let_script_runs:
            self._send_velocity_command_using_radius(0)
            rate.sleep()
            
        self.stop_index_publisher.publish(999999)

    def move_until_beginning_of_path(self, speed_goal, speed_rate):
        if not let_script_runs:
            return
        self._display_message('Executing move_until_beginning_of_path')
        self._display_message(dash_line)
        rate = rospy.Rate(50)

        initial_time = time.time()
        initial_speed = self.limiter_initial_speed
        while (self.current_path_index > 0) and let_script_runs:
            limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)
            self._send_velocity_command_using_radius(limited_speed)
            rate.sleep()

    def sleep(self, seconds):
        if not let_script_runs:
            return
        self._display_message('Executing sleep')
        time.sleep(seconds)

    def _get_time_now_in_ms(self):
        epoch = datetime.utcfromtimestamp(0)
        now = datetime.utcnow()
        delta = now - epoch
        return delta.total_seconds() * 1000

    def go_straight_for_ms(self, speed, milliseconds):
        if not let_script_runs:
            return
        self._display_message('Executing go_straight_for_ms')
        start_time = self._get_time_now_in_ms()
        pose2d = Pose2D()
        pose2d.x = speed
        pose2d.theta = 0

        rate = rospy.Rate(10)
        while (self._get_time_now_in_ms() - start_time < milliseconds) and let_script_runs:
            self.velocity_command_publisher.publish(pose2d)
            rate.sleep()

    def rotate_for_ms(self, angular_speed, milliseconds):
        if not let_script_runs:
            return
        self._display_message('Executing rotate_for_ms')
        start_time = self._get_time_now_in_ms()
        pose2d = Pose2D()
        pose2d.x = 0
        pose2d.theta = -angular_speed

        rate = rospy.Rate(10)
        while (self._get_time_now_in_ms() - start_time < milliseconds) and let_script_runs:
            self.velocity_command_publisher.publish(pose2d)
            rate.sleep()

        pose2d.x = 0
        pose2d.theta = 0
        self.velocity_command_publisher.publish(pose2d)

    def wait_for_target_position(self, trigger_lat, trigger_long, trigger_heading):
        """
        Spin until the target passes the trigger location.

        Parameters:
            trigger_lat - latitude in degrees of trigger location
            trigger_long - longitude in degrees of trigger location
            trigger_heading - heading in degrees of trigger location
        Return:
            n/a
        """
        if not let_script_runs:
            return
        self._display_message('Executing wait_for_target_position')
        llne = LL_NE(trigger_lat, trigger_long)
        boundary_checker = CheckBoundariesEnter(trigger_heading)

        rate = rospy.Rate(50)

        while self.target_gps_ready and let_script_runs:
            py, px = llne.LL2NE(self.target_latitude, self.target_longitude)
            if boundary_checker.in_boundaries(np.array([px,py])):
                if abs((trigger_heading - self.target_heading) < 45):
                    break
                else:
                    #TO-DO: target vehicle is not at the correct direction
                    print("Vehicle approaches at wrong direction, handle this error. End test.")
                    break
            rate.sleep()

        if not self.target_gps_ready:
            #TO-DO: test failed due to target gps not valid, implement proper safety measure
            print("ERROR: Something wrong. Target GPS not ready, handle this error. End test.")

    def wait_for_target_velocity(self, velocity):
        if not let_script_runs:
            return
        self._display_message('Executing wait_for_target_velocity')
        rate = rospy.Rate(20)
        while (self.target_velocity < velocity 
            and self.target_gps_ready) and let_script_runs:
            rate.sleep()
        
        if not self.target_gps_ready:
            #TO-DO: test failed due to target gps not valid, implement proper safety measure
            print("ERROR: Something wrong. Target GPS not ready, handle this error.End test.")

    # Subscriber Callbacks
    def current_path_index_callback(self, msg):
        self.current_path_index = msg.data

    def max_path_index_callback(self, msg):
        self.max_path_index = msg.data

    def path_intervals_callback(self, msg):
        self.path_intervals = msg.data

    def gps_callback_1(self, msg):
        self.robot_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        self.robot_angular_speed = abs(msg.angular.z)

    def gps_callback_2(self, msg):
        self.robot_heading = msg.data # in radian

    def gps_velocity_callback(self, msg):
        self.robot_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def gps_orientation_callback(self, msg):
        self.robot_heading = msg.z

    def gps_sbg_euler_callback(self, msg):
        self.robot_heading = msg.angle.z

    def gps_sbg_nav_callback(self, msg):
        self.robot_speed = math.sqrt(msg.velocity.x**2 + msg.velocity.y**2)

    def target_callback(self, msg):
        self.target_heading = msg.heading
        self.target_velocity = msg.velocity
        self.target_gps_ready = msg.gps_ready
        self.target_longitude = msg.longitude
        self.target_latitude = msg.latitude
        self.target_gps_correction_type = msg.gps_correction_type

    def turning_radius_callback(self, msg):
        self.turning_radius = msg.data

class Receptionist:
    def __init__(self):
        self.script_folder = "../../../custom_script/"
        self.filename = ""
        self.is_script_running = False
        self.previous_state = -1
        self.is_script_okay = False
        self.overseer_state = 0

        # Publishers
        self.is_script_running_publisher = rospy.Publisher('/robot_commander/is_script_running', Bool, queue_size=10)
        self.script_name_publisher = rospy.Publisher('/path_follower/script_name', String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)
        rospy.Subscriber('/gui/upload_script_clicked', Empty, self.upload_script_clicked_callback)

        self.is_script_running_publisher.publish(False)
        self.check_script()

    def check_script(self):

        # !!!! Need to add syntax checking !!!!
        self.is_script_okay = True
        # !!!!

        if os.path.exists(self.script_folder):
            py_files = glob(self.script_folder + '*.py')
            if len(py_files) == 1:
                filepath = py_files[0]   

                self.filename = os.path.basename(filepath)
                self.script_name_publisher.publish(self.filename)

    def start_custom_script(self):
        if self.is_script_okay:
            try:
                execfile(self.script_folder + self.filename)
            except Exception as error:
                print(error)

        self.is_script_running = False
        self.is_script_running_publisher.publish(False) # This will change the state in overseer.py to STOP
        
        end_message = "Test ended"
        print(end_message)
        command_message_publisher.publish(end_message)
        command_message_publisher.publish(dash_line) # needed for separating tests

    def return_to_start(self):
        try:
            execfile('./return_to_start.py')
        except Exception as error:
            print(error)

        self.is_script_running = False
        self.is_script_running_publisher.publish(False) # This will change the state in overseer.py to STOP

    def overseer_state_callback(self, msg):
        self.overseer_state = msg.data

    def upload_script_clicked_callback(self, msg):
        self.check_script()

if __name__ == '__main__':
    node = rospy.init_node('robot_commander')
    command_message_publisher = rospy.Publisher('/robot_commander/command_message', String, queue_size=20)

    recept = Receptionist()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # if there is a state change
        if recept.previous_state != recept.overseer_state:
            if recept.overseer_state == AUTO and not recept.is_script_running:
                recept.is_script_running = True
                let_script_runs = True
                recept.is_script_running_publisher.publish(True)

                start_message = "Test started"
                print(start_message)
                command_message_publisher.publish(start_message)

                custom_script_thread = threading.Thread(target=recept.start_custom_script)
                custom_script_thread.setDaemon(True)
                custom_script_thread.start()
            elif recept.overseer_state == RETURN_TO_START and not recept.is_script_running:
                recept.is_script_running = True
                let_script_runs = True
                recept.is_script_running_publisher.publish(True)

                custom_script_thread = threading.Thread(target=recept.return_to_start)
                custom_script_thread.setDaemon(True)
                custom_script_thread.start()

            # If the STOP button is clicked when the custom script is still running, kill this ROS node by breaking out of the while loop.
            # This ROS node will respawn after being killed
            elif (recept.overseer_state != AUTO and recept.overseer_state != RETURN_TO_START) \
                                                and recept.is_script_running:
                let_script_runs = False
                recept.is_script_running_publisher.publish(False)
                # break

            recept.previous_state = recept.overseer_state
            
        rate.sleep()