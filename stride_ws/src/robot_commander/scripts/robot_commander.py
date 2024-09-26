#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from __future__ import division
from socket import setdefaulttimeout
import rospy
import time
import threading
import os
import math
import numpy as np
from glob import glob
from shared_tools.libcal import LL_NE, CheckBoundariesEnter, \
    Compensation, Compensation_Errors,\
    kph2mps, mps2kph
from std_msgs.msg import Int32, Empty, Bool, String, Float32, Float32MultiArray
from geometry_msgs.msg import Pose2D, Twist, TwistWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav
from external_interface.msg import TargetVehicle
from path_follower.msg import Latlong
from datetime import datetime
from shared_tools.utils import find_rate_limited_speed as _find_rate_limited_speed
from shared_tools.utils import WriteCSV
from shared_tools.overseer_states_constants import *
from check_script import *
from math import cos, pi, sin

# Global control robot commander across threads

let_script_runs = False
dash_line = "----------------------"

class RobotCommander(object):
    def __new__(cls):
        """
        Singleton - keep only one object running for the entire node
        if the object is already created. Return that object
        """
        if not hasattr(cls,'instance'):
            cls.instance = super(RobotCommander, cls).__new__(cls)
            cls.instance.__initialized = True
            cls.instance.__testing = False
        else:
            cls.instance.__testing = True

        return cls.instance
    
    def __init__(self):
        self.current_path_index = 0
        self.max_path_index = -1
        self.path_intervals = []
        self.robot_speed = -1
        self.robot_heading = -1
        self.turning_radius = 999
        self.limiter_initial_speed = 0
        self.brake_command = 0
        
        self.pressure_switch_status = 0
        self.target_latitude = 0
        self.target_longitude = 0
        self.stride_dtc = Compensation_Errors().INVALID

        if self.__initialized:
            self.has_brake          = rospy.get_param('has_brake', False)
            self.reverse_speed_goal = rospy.get_param('~reverse_speed_goal', -1.5)
            self.default_decel_rate = rospy.get_param('decel_rate', 0.1)
            self.default_accel_rate = rospy.get_param('accel_rate', 0.1)
            self.debug              = rospy.get_param('debug', False)
            
            self.target_gps_ready = False
            self.vehicle_comp = False

            # Publishers
            self.velocity_command_publisher = rospy.Publisher('/robot_velocity_command', Pose2D, queue_size=1)
            self.stop_index_publisher = rospy.Publisher('/robot_commander/stop_index', Int32, queue_size=1)
            self.spin_velocity_publisher = rospy.Publisher('/robot_commander/spin_in_place_velocity', Float32, queue_size=1)
            self.set_index_publisher = rospy.Publisher('/robot_commander/index_to_be_set', Int32, queue_size=1)
            self.disable_motor_publisher = rospy.Publisher('/robot_commander/disable_motor', Bool, queue_size=1)
            self.brake_command_publisher = rospy.Publisher('/brake_command', Bool, queue_size = 1)
            self.collision_point_publisher = rospy.Publisher('/robot_commander/collision_point', Latlong, queue_size=1)
            self.stride_dtc_publisher = rospy.Publisher('/robot_commander/distance_to_collision', Float32, queue_size=1)

            self.sub = {}
            # Subscribers
            self.sub['target'] = rospy.Subscriber('/target', TargetVehicle, self.target_callback, queue_size=1)
            self.sub['pressure_switch'] = rospy.Subscriber('/pressure_switch', Bool, self.pressure_switch_callback, queue_size=1)

            self.sub['sbg_ekf_euler'] = rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.gps_sbg_euler_callback, queue_size=1)
            self.sub['sbg_ekf_nav'] = rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.gps_sbg_nav_callback, queue_size=1)

            self.brake_status = 3
            if self.has_brake:
                self.sub['brake_status'] = rospy.Subscriber('/brake_status', Int32, self.brake_status_callback, queue_size=1)
                self.sub['left_brake'] = rospy.Subscriber('/fullyseated_L', Int32, self.left_brake_callback, queue_size=1)
                self.sub['right_brake'] = rospy.Subscriber('/fullyseated_R', Int32, self.right_brake_callback, queue_size=1)
            else:
                self.brake_status = 1 # wheels are not blocked status
            self.__initialized = False
        else:
            if not self.has_brake:
                self.brake_status = 1

        # Latched subscribers, re-subscribe every test
        if hasattr(self, 'sub'):
            self.sub['current_path_index'] = rospy.Subscriber('/path_follower/current_path_index', Int32, self.current_path_index_callback)
            self.sub['max_path_index'] = rospy.Subscriber('/path_follower/max_path_index', Int32, self.max_path_index_callback)
            self.sub['path_intervals'] = rospy.Subscriber('/path_follower/path_intervals', Float32MultiArray, self.path_intervals_callback)
            self.sub['turning_radius'] = rospy.Subscriber('/path_follower/turning_radius', Float32, self.turning_radius_callback)
            self.sub['path_to_follow'] = rospy.Subscriber('/path_follower/path_to_follow', Latlong, self.path_to_follow_callback)

        # blocking until these attributes have been updated by subscriber callbacks
        if self.__testing:            
            while (self.max_path_index == -1 or self.path_intervals == [] or self.robot_speed == -1 or self.robot_heading == -1 or self.turning_radius == 999):
                time.sleep(0.01)

    def _display_message(self, message):
        if self.debug:
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

    def _rate_limiter(self, speed_goal, acceleration, total_distance_in_index):
        acceleration = acceleration * 9.81
        initial_time_in_s = time.time()
        current_velocity = 0
        is_starting_decel = True
        frequency = 100 #hz
        period = 1/frequency
        r = rospy.Rate(frequency) 
        minimum_decel_vel = 0.3 #m/s
       
        if speed_goal < 0: #reverse motion
            acceleration_distance =  total_distance_in_index*2/3 
            const_vel_distance = total_distance_in_index/3

            while self.current_path_index > 0 and let_script_runs:
                if self.current_path_index >=  acceleration_distance:
                    velocity_input = math.copysign((min(abs(acceleration * (time.time()-initial_time_in_s)), abs(speed_goal))), speed_goal)
                elif self.current_path_index >=  const_vel_distance:
                    velocity_input =  current_velocity
                else:
                    if is_starting_decel: 
                        d = sum(self.path_intervals[0 : self.current_path_index])
                        a = current_velocity**2 / (2*d) #vf^2 = vi^2 + 2*a*d
                        is_starting_decel = False

                    velocity_input = math.copysign(max(abs(current_velocity + a*period) , abs(minimum_decel_vel)), speed_goal) #current velocity is -ve
                self._send_velocity_command_using_radius(velocity_input)
                r.sleep() 
                current_velocity = velocity_input
                
        else: #forward motion
            distance_to_be_covered = total_distance_in_index-self.current_path_index 
            acceleration_distance = self.current_path_index + distance_to_be_covered/3
            const_vel_distance = self.current_path_index + distance_to_be_covered*2/3

            while self.current_path_index < total_distance_in_index and let_script_runs:
                if self.current_path_index <=  acceleration_distance:
                    velocity_input = min(acceleration * (time.time()-initial_time_in_s), speed_goal)
                elif self.current_path_index <=  const_vel_distance:
                    velocity_input = current_velocity
                else:
                    if is_starting_decel: 
                        d = sum(self.path_intervals[self.current_path_index : total_distance_in_index])
                        a = current_velocity**2 / (2*d) 
                        is_starting_decel = False

                    velocity_input = max(current_velocity - a*period, minimum_decel_vel) 
                self._send_velocity_command_using_radius(velocity_input)
                r.sleep() 
                current_velocity = velocity_input

    def _rate_limit_to_distance(self, speed_goal, acceleration, distance_goal, total_distance_in_index): #For accel_to_distance() motion.
        acceleration = acceleration * 9.81
        initial_time_in_s = time.time()
        current_velocity = 0
        is_starting_decel = True
        frequency = 50 #hz
        period = 1/frequency
        r = rospy.Rate(frequency) 
        minimum_decel_vel = 0.3 #m/s

        index_dist = distance_goal / 0.3
       
        #Forward motion
        distance_to_be_covered = total_distance_in_index-self.current_path_index 
        acceleration_distance = self.current_path_index + (index_dist - self.current_path_index)
        const_vel_distance = self.current_path_index + distance_to_be_covered*2/3

        while self.current_path_index < total_distance_in_index and let_script_runs:
            if self.current_path_index <=  acceleration_distance:
                velocity_input = min(acceleration * (time.time()-initial_time_in_s), speed_goal)
            elif self.current_path_index <=  const_vel_distance:
                velocity_input = current_velocity
            else:
                if is_starting_decel: 
                    d = sum(self.path_intervals[self.current_path_index : total_distance_in_index])
                    a = current_velocity**2 / (2*d) 
                    is_starting_decel = False

                velocity_input = max(current_velocity - a*period, minimum_decel_vel) 
            self._send_velocity_command_using_radius(velocity_input)
            r.sleep() 
            current_velocity = velocity_input

    
    def _LL2NE(self, RefLat, RefLong, latitude, longitude): #Some conversion formula obtained from GeneSys documentation
        e = 0.0818191908426; #Some constant
        R = 6378137; #Some constant

        #Scale factor for longitude (deg) to East (m)
        Efactor = cos(RefLat*pi/180)*pi/180*R/ np.sqrt(1-np.square((sin(RefLat*pi/180)))* e ** 2)
        #Scale factor for latitude (deg) to North (m)
        Nfactor = (1-e**2)*R/((1-(np.square(sin(RefLat*pi/180))*e**2))*np.sqrt(1-(np.square(sin(RefLat*pi/180))*e**2)))*pi/180

        col1 = Efactor * (longitude - RefLong)  #Apply east scale factor and put into column variable
        col2 = Nfactor * (latitude - RefLat)    #Apply north scale factor and put into column variable
        return col1, col2   #Return values from each column as their own variable

    def move_until_end_of_path(self, speed_goal, speed_rate): #Units are meters/second and g
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing move_until_end_of_path')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return

        self._rate_limiter(speed_goal, speed_rate, self.max_path_index)

    def brake_to_stop(self, speed_rate): #Unit is g
        global let_script_runs
        speed_rate = speed_rate * 9.81
        if not let_script_runs:
            return
        self._display_message('Executing brake_to_stop')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
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

    def move_until_index(self, speed_goal, speed_rate, index): #Units are meters/second, g, and unitless
        speed_rate = speed_rate * 9.81
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing move_until_index')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
        if index > self.max_path_index:
            let_script_runs = False
            self._display_message("Aborting Test: Input index must not exceed max path index.")
            return
        elif index < self.current_path_index:
            let_script_runs = False
            self._display_message('Aborting Test: Attempting to move to index that has already been passed.')
            return
        rate = rospy.Rate(50)
        initial_time = time.time()
        initial_speed = self.limiter_initial_speed
        while (self.current_path_index < index) and let_script_runs:
            limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed)
            self._send_velocity_command_using_radius(limited_speed)
            rate.sleep()

    def accel_to_distance(self, speed_goal, distance_goal, P_gain = 1): #Units are kilometers/hour, meters, and unitless
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing accel_to_distance')
        speed_goal = speed_goal * (1000/3600) #Convert km/hr to m/s
        acceleration_mps2 = np.square(speed_goal) / (2 * distance_goal* P_gain)
        acceleration_g = acceleration_mps2 / 9.81
        index_dist = distance_goal / 0.3
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
        if speed_goal < 0 or distance_goal < 0: #If entered speed goal is negative
            let_script_runs = False
            self._display_message('Aborting Test: Please enter a positive value for the speed and distance goals.')
            return

        if self.vehicle_comp == True:
            self._rate_limit_to_distance(speed_goal, acceleration_g, distance_goal, index_dist)
        else:
            self._rate_limit_to_distance(speed_goal, acceleration_g, distance_goal, self.max_path_index)

    # maybe add a try-except statement to catch zero angular velocity and zero tolerance
    def rotate_until_heading(self, angular_velocity, heading, heading_tolerance = 3): #Units are degrees/second, degrees, and degrees
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing rotate_until_heading')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
        if heading >= 0:
            heading = heading % 360
        else:
            heading = heading % -360

        heading_radian = heading / 180 * math.pi
        tolerance_radian = heading_tolerance / 180 * math.pi

        if heading > 180 and heading <360: #Map heading between 180 and 360 degrees to heading from 0 to -180 degrees
            heading = heading - 360
        elif heading < -180 and heading > -360: #Map heading between -180 and -360 degress to heading from 0 to 180 degrees
            heading = heading + 360
        
        #Find lower and upper bounds for desired heading
        if heading >=3: 
            lower_bound = (heading_radian - tolerance_radian) % (2*math.pi)
            upper_bound = (heading_radian + tolerance_radian) % (2*math.pi)
        elif heading < 3 and heading > -3:
            lower_bound = (heading_radian - tolerance_radian) 
            upper_bound = (heading_radian + tolerance_radian) 
        else:
            lower_bound = (heading_radian - tolerance_radian) % -(2*math.pi)
            upper_bound = (heading_radian + tolerance_radian) % -(2*math.pi)

        angular_velocity = angular_velocity * (np.pi / 180)
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

    def decel_to_stop_at_index(self, stop_index): #Unitless
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing decel_to_stop_at_index')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
        if stop_index > self.max_path_index:
            let_script_runs = False
            self._display_message("Aborting Test: Input stop index must not exceed max path index.")
            return
        elif stop_index < self.current_path_index:
            let_script_runs = False
            self._display_message('Aborting Test: Attempting to stop at index that has already been passed.')
            return
        
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

    def move_until_beginning_of_path(self, speed_goal, speed_rate): #Units are meters/second and g
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing move_until_beginning_of_path')
        if self.brake_status != 1: #Block function if brake isn't fully disengaged
            let_script_runs = False
            self._display_message("Aborting Test: Brake not disengaged.")
            return
        self._display_message(dash_line)
        self._rate_limiter(speed_goal, speed_rate, self.current_path_index)

    def sleep(self, seconds): #Unit is seconds
        if not let_script_runs:
            return
        self._display_message('Executing sleep')
        rate = rospy.Rate(50)
        t0 = time.time()
        time_check = seconds
        while let_script_runs:
            if ((time.time() - t0) > time_check):
                break
            rate.sleep()

    def engage_brake_hill(self): #No arguments needed
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing engage_brake')

        if not self.has_brake:
            self._display_message('Aborting Test: Is this a brake-supported platform? Check parameters.')
            self.brake_to_stop(self.default_decel_rate)
            let_script_runs = False
            return

        rate = rospy.Rate(50)

        #Pitch check
        if abs(self.pitch) < 4 /180*math.pi:
            self._display_message("Aborting Test: Pitch not great enough to engage brake.") #Print to GUI 
            let_script_runs = False
            return        

        #Send speed to zero before braking
        while self.robot_speed > 0.1 and let_script_runs:
            self._display_message("WARNING: Robot speed still active before brake")
            self.brake_to_stop(0.1)
            rate.sleep()

        # Tell Arduino via UDP to engage brake 
        self.brake_command = True
        self.brake_command_publisher.publish(self.brake_command)

        #Timeout info
        engage_brake_timeout = False
        timeout = 50 
        t0 = time.time()
        
        #While loop to block code until Arduino says brake is engaged via UDP 
        time.sleep(0.2)
        t1 = time.time()
        time_check = 5 #Time for motors to be able to relax
        while self.brake_status != 2 and let_script_runs: 
            if (time.time() - t0) > timeout: #Timeout engage brake when it fails
                engage_brake_timeout = True
                break

            if self.brake_status == 2: #Exit while loop if brake is fully engaged
                break
            elif (time.time() - t1) > time_check: #If rollback doesn't engage brake, disengage then reengage brake
                self.brake_command_publisher.publish(False)
                time.sleep(1)
                self.brake_command_publisher.publish(True)
                t1 = time.time()
            rate.sleep()

        if self.brake_status == 2: #Once brake fully engaged, wait 0.5 seconds and then disable motors
            time.sleep(0.5)  
            self.disable_motor_publisher.publish(True) 
        elif engage_brake_timeout == True: #If timeout occurs
            let_script_runs = False #Abort test. State will be STOPPED
            self._display_message("Aborting Test: Engage brake failed") #Warning message to gui 

    def disengage_brake_hill(self): #No arguements needed
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing disengage_brake')

        if not self.has_brake:
            self._display_message('Aborting Test: Is this a brake-supported platform? Check parameters.')
            self.brake_to_stop(self.default_decel_rate)
            let_script_runs = False
            return

        rate = rospy.Rate(50)

        #Enable motors before disengaging
        self.disable_motor_publisher.publish(False)
        time.sleep(0.1)

        #Tell arduino to disengage brake
        self.brake_command = False 
        self.brake_command_publisher.publish(self.brake_command)

        #Timeout info
        disengage_brake_timeout = False
        timeout = 2
        t0 = time.time()

        #While loop to block code until Ardino says brake is disengaged via UDP 
        while self.brake_status != 1 and let_script_runs:
            rate.sleep()
            self._send_velocity_command_using_radius(0.025)
            if (time.time() - t0) > timeout: #Timeout disengage brake when it fails
                disengage_brake_timeout = True
                break

        if self.brake_status == 1: #Exit function if brake fully disengaged
            self._send_velocity_command_using_radius(0)
            return
        elif disengage_brake_timeout == True: #If function times out, abort test and send message to the gui
            self._send_velocity_command_using_radius(0)
            let_script_runs = False #Abort test
            self._display_message('Aborting Test: Disengage brake failed. User action required.') #Send message to GUI to let user know they need to do something.

    def wait_for_vehicle_position(self, trigger_lat, trigger_long, trigger_heading): #Units are degrees, degrees, and degrees
        """
        Spin until the target passes the trigger location.

        Parameters:
            trigger_lat - latitude in degrees of trigger location
            trigger_long - longitude in degrees of trigger location
            trigger_heading - heading in degrees of trigger location
        Return:
            n/a
        """
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing wait_for_vehicle_position')
        llne = LL_NE(trigger_lat, trigger_long)
        boundary_checker = CheckBoundariesEnter(trigger_heading)

        rate = rospy.Rate(50)

        # time.sleep(0.01)
        while self.target_gps_ready and let_script_runs:
            py, px = llne.LL2NE(self.target_latitude, self.target_longitude)
            if boundary_checker.in_boundaries(np.array([px,py])):
                if abs((trigger_heading - self.target_heading) < 45):
                    break
                else:
                    #TO-DO: target vehicle is not at the correct direction
                    self._display_message("Aborting Test: Vehicle approaches at wrong direction.")
                    let_script_runs = False
                    return
            rate.sleep()

        if not self.target_gps_ready:
            #TO-DO: test failed due to target gps not valid, implement proper safety measure
            self._display_message("Aborting Test: Target GPS is not ready.")
            let_script_runs = False
            return
        
    def pressure_switch(self):
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing pressure_switch')

        rate = rospy.Rate(50)

        while let_script_runs:
            if self.pressure_switch_status == 1:
                return
            rate.sleep()

    def wait_for_vehicle_velocity(self, velocity): #Unit is meters/second
        global let_script_runs
        if not let_script_runs:
            return
        self._display_message('Executing wait_for_vehicle_velocity')
        rate = rospy.Rate(50)
        # time.sleep(0.01)
        while (self.target_velocity < velocity 
            and self.target_gps_ready) and let_script_runs:
            rate.sleep()
        
        if not self.target_gps_ready:
            #TO-DO: test failed due to target gps not valid, implement proper safety measure
            self._display_message("Aborting Test: Target GPS is not ready.")
            let_script_runs = False
            return

    def vehicle_compensation(self, speed_goal_kph, speed_rate_g, intersection_lat, intersection_long, \
                             accel_dist, speed_tolerance=0.2, \
                             vehicle_intersection_lat=None, vehicle_intersection_long=None): 
            """
            Units are kph, g, degrees, degrees, meters, kph, deg, deg
            """
            global let_script_runs
            if not let_script_runs:
                return
            self._display_message('Executing vehicle_compensation')
            if self.brake_status != 1: #Block function if brake isn't fully disengaged
                let_script_runs = False
                self._display_message("Aborting Test: Brake not disengaged.")
                return
            
            # Compensation setup
            if vehicle_intersection_lat == None or vehicle_intersection_long == None:
                vehicle_intersection_lat = intersection_lat
                vehicle_intersection_long = intersection_long
            msg = Latlong()
            msg.latitudes = [vehicle_intersection_lat]
            msg.longitudes = [vehicle_intersection_long]
            self.collision_point_publisher.publish(msg)
            stride_comp = Compensation(self.path_to_follow)

            if not stride_comp.pre_collision_calc(intersection_lat, intersection_long):
                let_script_runs = False
                self._display_message("Aborting Test: comp pre-reqs do not meet")
                return

            #Initialize variables
            speed_rate = speed_rate_g * 9.81 #Convert g to m/s^2
            minimum_velocity = 0.1 #m/s
            euro_ncap_speed_tol = kph2mps(speed_tolerance) #0.2 kph speed tolerance to m/s 
            speed_goal_mps = kph2mps(speed_goal_kph)
            lower_vel_threshold = max(speed_goal_mps - euro_ncap_speed_tol, minimum_velocity)
            upper_vel_threshold = speed_goal_mps + euro_ncap_speed_tol
            exceed_speed_goal = False

            if not self.target_gps_ready:
                self._display_message("Aborting Test: Target GPS is not ready.")
                let_script_runs = False
                return
            else:
                speed_rate_for_dist = (speed_goal_mps**2 - self.limiter_initial_speed**2) / (2*accel_dist)
                initial_time = time.time()
                initial_speed = self.limiter_initial_speed
                limited_speed = initial_speed

                # accel_time = (speed_goal_mps - initial_speed)/speed_rate_for_dist
                # accel_to_max_speed_time = (upper_vel_thershold - speed_goal_mps) / speed_rate
                # leftover_dist = accel_dist + (upper_vel_threshold**2 - speed_goal_mps**2)/(2*speed_rate)
                # stride_dtc = stride_comp.dist_to_collision(self.stride_latitude, self.stride_longitude,self.current_path_index)
                # max_speed_comp_time = (stride_dtc - leftover_dist) / upper_vel_threshold
                # stride_ttc = accel_time + max_speed_comp_time
                # if stride_ttc > sv_ttc:
                #   abort operation, can't catch up, there is no point

                rate = rospy.Rate(100)
                # Accel until exceed speed goal
                while self.current_path_index < self.max_path_index and let_script_runs and not exceed_speed_goal:
                    if limited_speed < speed_goal_mps:
                        limited_speed = _find_rate_limited_speed(speed_rate_for_dist, initial_time, speed_goal_mps, initial_speed)
                        self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                    else:
                        exceed_speed_goal = True
                    rate.sleep()

                initial_time = time.time()
                limited_speed = self.limiter_initial_speed

                # Compensate
                while self.current_path_index < self.max_path_index and let_script_runs:                    
                    #Calculate time to collision point.
                    self.stride_dtc = stride_comp.dist_to_collision(self.stride_latitude, self.stride_longitude,\
                                                        self.current_path_index)
                    if self.stride_dtc < Compensation_Errors().PAST_TRIGGER_POINT or \
                            self.target_dtc < Compensation_Errors().PAST_TRIGGER_POINT:
                        let_script_runs = False
                        self._display_message("Aborting Test: collision calc stride dtc {}, target dtc {}".format(self.stride_dtc, self.target_dtc))
                        return
                    
                    delta = 0.0
                    stride_ttc = float(np.round(self.stride_dtc / self.robot_speed, 2))
                    sv_ttc = float(np.round((self.target_dtc + delta)/ self.target_velocity, 2))

                    # TO-DO: Add check

                    if stride_ttc < 0:
                        limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal_mps, limited_speed)
                        # self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                    elif stride_ttc < sv_ttc: #Lower Stride's speed
                        limited_speed = _find_rate_limited_speed(speed_rate, initial_time, lower_vel_threshold, limited_speed)
                        # self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                    elif stride_ttc > sv_ttc: #Raise Stride's speed
                        limited_speed = _find_rate_limited_speed(speed_rate, initial_time, upper_vel_threshold, limited_speed)
                        # self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                    else: #Keep Stride's last speed
                        # limited_speed = _find_rate_limited_speed(speed_rate, initial_time, speed_goal_mps, limited_speed)
                        # self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                        pass

                    self._send_velocity_command_using_radius(max(limited_speed, minimum_velocity))
                    initial_time = time.time()
                    self.stride_dtc_publisher.publish(self.stride_dtc)
                    rate.sleep()

    def euroncap_cpfa(self, intersection_lat, intersection_long):
        """
        Euro NCAP Test for Car-to-Pedestrian Farside Adult
        impact position 50%
        """
        speed_goal = 8.0 # km/h
        accel_dist = 1.5 # meters
        self._display_message("Start Euroncap test")
        self.vehicle_compensation(speed_goal, 0.1, \
                                  intersection_lat, intersection_long, accel_dist)

    def euroncap_cpna(self, intersection_lat, intersection_long):
        """
        Euro NCAP Test for Car-to-Pedestrian Farside Adult
        impact position 50%
        """
        speed_goal = 5.0 # km/h
        accel_dist = 1.0 # meters

        self.vehicle_compensation(speed_goal, 0.1, \
                                  intersection_lat, intersection_long, accel_dist)

    def custom_compensation(self, inter_lat, inter_long, veh_inter_lat, veh_inter_long, speed_kph, speed_rate_g, accel_dist, speed_tol=0.2):
        """
        A custom function for robot compensation to vehicle. Can adjust all params
        """
        self._display_message("Start compensation")
        self.vehicle_compensation(speed_kph, speed_rate_g, \
                                  inter_lat, inter_long, accel_dist, speed_tol, \
                                    veh_inter_lat, veh_inter_long)
        
    # Subscriber Callbacks
    def current_path_index_callback(self, msg):
        self.current_path_index = msg.data

    def max_path_index_callback(self, msg):
        self.max_path_index = msg.data

    def path_intervals_callback(self, msg):
        self.path_intervals = msg.data

    def gps_sbg_euler_callback(self, msg):
        self.robot_heading = msg.angle.z
        self.pitch = msg.angle.y

    def gps_sbg_nav_callback(self, msg):
        # self.robot_speed = math.sqrt(msg.velocity.x**2 + msg.velocity.y**2)
        self.robot_speed = msg.velocity.x * math.cos(self.robot_heading) + msg.velocity.y * math.sin(self.robot_heading)
        self.robot_speed_lat = msg.velocity.y * math.sin(self.robot_heading) + msg.velocity.y * math.cos(self.robot_heading)
        
        self.stride_latitude = msg.latitude
        self.stride_longitude = msg.longitude

    def target_callback(self, msg):
        self.target_heading = msg.heading
        self.target_velocity = msg.velocity
        self.target_gps_ready = msg.gps_ready
        self.target_longitude = msg.longitude
        self.target_latitude = msg.latitude
        self.target_gps_correction_type = msg.gps_correction_type
        self.target_dtc = msg.distance_to_collision

    def turning_radius_callback(self, msg):
        self.turning_radius = msg.data

    def path_to_follow_callback(self, msg):
        self.path_to_follow = {}
        self.path_to_follow['latitudes'] = msg.latitudes
        self.path_to_follow['longitudes'] = msg.longitudes

    def brake_status_callback(self, msg):
        self.brake_status = msg.data

    def left_brake_callback(self, msg):
        self.fully_seated_L = msg.data

    def right_brake_callback(self, msg):
        self.fully_seated_R = msg.data

    def pressure_switch_callback(self,msg):
        self.pressure_switch_status = msg.data

class Receptionist:
    def __init__(self):
        self.script_folder = "../../../custom_script/"
        self.filename = ""
        self.is_script_running = False
        self.previous_state = -1
        self.is_script_okay = False
        self.overseer_state = 0

        self.brake = rospy.get_param('has_brake', False)

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
        result = "OK"
        # !!!!

        if os.path.exists(self.script_folder):
            py_files = glob(self.script_folder + '*.py')
            if len(py_files) == 1:
                filepath = py_files[0]   

                self.filename = os.path.basename(filepath)
                result = check_script(self.script_folder + self.filename, self.brake)
                if result == "OK":
                    self.is_script_okay = True
                    command_message_publisher.publish("Script is OK to run.")
                else:
                    self.is_script_okay = False
                    command_message_publisher.publish(result)
                self.script_name_publisher.publish(self.filename)

    def start_custom_script(self):
        if self.is_script_okay:
            try:
                execfile(self.script_folder + self.filename)
            except Exception as error:
                print(error)
        else:
            error_message = "ERROR: Invalid test script. Abort test."
            command_message_publisher.publish(error_message)
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

    rc = RobotCommander()

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