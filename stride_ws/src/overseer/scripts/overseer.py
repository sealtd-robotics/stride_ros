#!/usr/bin/env python

# Abbreviations
# mc: motor controller
# mcs: motor controllers (as a list)
# lf: left front
# lb: left back
# rf: right front
# rb: right back

from __future__ import division
import glob
import os
import math
import rospy
from std_msgs.msg import Int32, Float32, Empty, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from sbg_driver.msg import SbgEkfEuler
import time
import enum
import threading
from datetime import datetime
from shared_tools.overseer_states_constants import *
from tf.transformations import euler_from_quaternion


class ErrorHandler:
    def __init__(self, mcs, gui, handheld):
        self.mcs = mcs
        self.gui = gui
        self.handheld = handheld

        # Publishers
        self.error_publisher = rospy.Publisher('/overseer/error_message', String, queue_size=20)

    def has_error(self, overseer_state, should_log_error):
        errors = ""
        has_error = False

        # Motor controllers
        for mc in self.mcs:
            error_word = mc.error_word
            if error_word != 0 and error_word != 64: # 64 is Hall sensor error. It comes up randomly and seems safe to ignore
                errors = errors + "{} error_word: {}\n".format(mc.name, error_word)
                has_error = True
            
            is_heartbeat_timeout = mc.is_heartbeat_timeout
            if is_heartbeat_timeout == True:
                errors = errors + "{} is_heartbeat_timeout: {}\n".format(mc.name, is_heartbeat_timeout)
                has_error = True
        
        # GUI
        is_gui_heartbeat_timeout = gui.is_heartbeat_timeout
        if is_gui_heartbeat_timeout == True:
            errors = errors + "GUI is_heartbeat_timeout: {}\n".format(is_gui_heartbeat_timeout)
            has_error = True

        # AUTO state
        if overseer_state == AUTO:
            pass # implement this later (for GPS error) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Log errors and inform GUI
        if has_error and should_log_error:
            # inform GUI
            self.error_publisher.publish(errors)

            # Log errors
            time = datetime.now().strftime("%H:%M:%S")
            errors = time + "\n" + errors + "\n"

            date = datetime.now().strftime("%Y_%m_%d")
            filename = 'error_log_{}.txt'.format(date)

            log_dir = '../../../error_log/'
            with open(log_dir + filename, 'a') as f:
                f.write(errors)

            # Remove old error log files
            files = glob.glob(log_dir + "error_log*")
            files.sort()
            files_to_keep = 60

            if len(files) > files_to_keep:
                for f in files[0:-files_to_keep]:
                    os.remove(f)
                    print('Removed log file "{}" as regular cleanup'.format(os.path.basename(f)))

        return has_error

class MotorController:
    def __init__(self, location, name):
        self.location = location
        self.name = name
        self.error_word = 0
        self.winding_temperature = 0
        self.is_heartbeat_timeout = False
        self.wheel_rpm_actual = 0

        rospy.Subscriber('/motor_controller/{}/error_word'.format(self.location), Int32, self.set_error_word)
        rospy.Subscriber('/motor_controller/{}/winding_temperature'.format(self.location), Int32, self.set_winding_temperature)
        rospy.Subscriber('/motor_controller/{}/is_heartbeat_timeout'.format(self.location), Bool, self.set_is_heartbeat_timeout)
        rospy.Subscriber('/motor_controller/{}/wheel_rpm_actual'.format(self.location), Float32, self.set_wheel_rpm_actual)

    def set_is_heartbeat_timeout(self, msg):
        self.is_heartbeat_timeout = msg.data

    def set_error_word(self, msg):
        self.error_word = msg.data

    def set_winding_temperature(self, msg):
        self.winding_temperature = msg.data

    def set_wheel_rpm_actual(self, msg):
        self.wheel_rpm_actual = msg.data

class Gui:
    def __init__(self):
        self.heartbeat_arrival_time = self.get_time_now_in_ms()
        self.is_stop_clicked = False
        self.is_enable_manual_clicked = False
        self.is_heartbeat_timeout = False
        self.is_start_following_clicked = False
        self.is_idle_clicked = False
        self.is_return_to_start_clicked = False

        rospy.Subscriber('/gui/stop_clicked', Empty, self.stop_callback)
        rospy.Subscriber('/gui/enable_manual_clicked', Empty, self.enable_manual_callback)
        rospy.Subscriber('/gui/heartbeat', Empty, self.heartbeat_callback, queue_size=1)
        rospy.Subscriber('/gui/start_path_following_clicked', Empty, self.start_following_callback)
        rospy.Subscriber('/gui/idle_clicked', Empty, self.idle_callback)
        rospy.Subscriber('/gui/return_to_start_clicked', Empty, self.return_to_start_callback)

        # Heartbeat timeout thread
        self.heartbeat_thread = threading.Thread(target=self.monitor_heartbeat)
        self.heartbeat_thread.setDaemon(True)
        self.heartbeat_thread.start()

    def monitor_heartbeat(self):
        while True:
            if self.get_time_now_in_ms() - self.heartbeat_arrival_time > 1000:
                self.is_heartbeat_timeout = True
            else:
                self.is_heartbeat_timeout = False
            time.sleep(0.1)

    # this is for handling burst of clicks in a slow network and hanlding buffered clicks not consumed by current state
    def reset_button_clicks(self):
        self.is_stop_clicked = False
        self.is_enable_manual_clicked = False
        self.is_start_following_clicked = False
        self.is_idle_clicked = False
        self.is_return_to_start_clicked = False

    def stop_callback(self, msg):
        self.is_stop_clicked = True

    def enable_manual_callback(self, msg):
        self.is_enable_manual_clicked = True

    def get_time_now_in_ms(self):
        epoch = datetime.utcfromtimestamp(0)
        now = datetime.utcnow()
        delta = now - epoch
        return delta.total_seconds() * 1000

    def heartbeat_callback(self, nmt_state_int):
        self.heartbeat_arrival_time = self.get_time_now_in_ms()

    def start_following_callback(self, msg):
        self.is_start_following_clicked = True

    def idle_callback(self, msg):
        self.is_idle_clicked = True

    def return_to_start_callback(self, msg):
        self.is_return_to_start_clicked = True

class Handheld:
    def __init__(self):
        self.is_estop_pressed = False

        self.is_estop_pressed_1 = False
        self.is_estop_pressed_2 = False
        rospy.Subscriber('/handheld/direct/is_estop_pressed', Bool, self.estop_callback_1)
        rospy.Subscriber('/handheld/through_xboard/is_estop_pressed', Bool, self.estop_callback_2)

    def estop_callback_1(self, msg):
        self.is_estop_pressed_1 = msg.data
        self.is_estop_pressed = self.is_estop_pressed_1 or self.is_estop_pressed_2

    def estop_callback_2(self, msg):
        self.is_estop_pressed_2 = msg.data
        self.is_estop_pressed = self.is_estop_pressed_1 or self.is_estop_pressed_2

class RobotCommander:
    def __init__(self):
        self.is_script_running = False
        rospy.Subscriber('/robot_commander/is_script_running', Bool, self.is_script_running_callback)

    def is_script_running_callback(self, msg):
        self.is_script_running = msg.data

class Gps:
    def __init__(self):
        self.pitch = 0
        rospy.Subscriber('/an_device/pitch', Float32, self.gps_callback_1, queue_size=1) # radian
        rospy.Subscriber('/gps/euler_orientation', Vector3, self.gps_callback_2, queue_size=1)
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.gps_euler_callback, queue_size=1)

    def gps_callback_1(self, msg):
        self.pitch = msg.data
        time.sleep(0.1)

    def gps_callback_2(self, msg):
        self.pitch = msg.y
        time.sleep(0.1)

    def gps_euler_callback(self, msg):
        self.pitch = msg.angle.z
        time.sleep(0.1)

# For Meredith's descend condition:
# Remember to Initialize this class in main!!!!!!!
# class Robot:
#     def __init__(self):
#         self.commanded_v = 0
#         rospy.Subscriber('/robot_velocity_command', Pose2D, self.velocity_command_callback, queue_size=1)      

#     def velocity_command_callback(self, pose2d):
#         self.commanded_v = pose2d.x


def should_descend(mcs, pitch):
    hot = False
    for mc in mcs:
        if mc.winding_temperature >= 105: # Set to less than 115C, which is the warning temperature. 
            hot = True

    should_descend = hot and abs(pitch) > 6 /180*math.pi
    return should_descend

# For Meredith's descend condition:
# Remember to change the arguments!!!!!!!!!
# def should_descend(mcs, pitch, commanded_v):
#     max_temp = 0
#     for mc in mcs:
#         max_temp = max(mc.winding_temperature, max_temp)
    
#     should_descend = False
#     if abs(pitch) > 6 /180*math.pi:
#         if max_temp > 115 or (max_temp > 105 and commanded_v == 0):
#             should_descend = True
    
#     return should_descend

def abs_max_wheel_rpm_actual(mcs):
    max_rpm = 0
    for mc in mcs:
        max_rpm = max(max_rpm, abs(mc.wheel_rpm_actual))
    return max_rpm

if __name__ ==  '__main__':
    node = rospy.init_node('overseer')

    # Motor Controllers
    mc_lf = MotorController('left_front', 'Motor Controller 1')
    mc_lb = MotorController('left_back', 'Motor Controller 2')
    mc_rf = MotorController('right_front', 'Motor Controller 3')
    mc_rb = MotorController('right_back', 'Motor Controller 4')
    mcs = [mc_lf, mc_lb, mc_rf, mc_rb]

    gui = Gui()
    handheld = Handheld()
    error_handler = ErrorHandler(mcs, gui, handheld)
    rc = RobotCommander()
    gps = Gps()
    # robot = Robot()

    # initial state
    state = STOPPED

    # Publishers
    state_publisher = rospy.Publisher('/overseer/state', Int32, queue_size=20)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Manual
        if state == MANUAL:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_start_following_clicked and not error_handler.has_error(state, False):
                state = AUTO
            elif gui.is_stop_clicked or error_handler.has_error(state, True) or should_descend(mcs, gps.pitch):
                state = STOPPED
            elif gui.is_return_to_start_clicked:
                state = RETURN_TO_START
        
        # Auto
        elif state == AUTO:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_stop_clicked or error_handler.has_error(state, True) or should_descend(mcs, gps.pitch):
                state = STOPPED
            elif not rc.is_script_running:
                # Making sure the rc.is_script_running has time to update
                time.sleep(0.1)
                if not rc.is_script_running:
                    state = STOPPED

        # E_Stopped
        elif state == E_STOPPED:
            if not handheld.is_estop_pressed:
                state = STOPPED

        # Stopped
        elif state == STOPPED:
            if gui.is_enable_manual_clicked and not error_handler.has_error(state, False):
                state = MANUAL
            elif gui.is_start_following_clicked and not error_handler.has_error(state, False):
                state = AUTO
            elif handheld.is_estop_pressed:
                state = E_STOPPED
            elif should_descend(mcs, gps.pitch) and abs_max_wheel_rpm_actual(mcs) < 50: # bring robot to stop before descending
                state = DESCENDING
            elif gui.is_idle_clicked:
                state = IDLE
            elif gui.is_return_to_start_clicked:
                state = RETURN_TO_START

        # Decending
        elif state == DESCENDING:
            if gui.is_idle_clicked or abs(gps.pitch) < 3/180*math.pi:
                state = IDLE

        # Idle
        elif state == IDLE:
            if gui.is_stop_clicked:
                state = STOPPED
            elif gui.is_enable_manual_clicked and not error_handler.has_error(state, False):
                state = MANUAL
            elif gui.is_start_following_clicked and not error_handler.has_error(state, False):
                state = AUTO
            elif handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_return_to_start_clicked:
                state = RETURN_TO_START

        # Return to start
        elif state == RETURN_TO_START:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_stop_clicked or error_handler.has_error(state, True) or should_descend(mcs, gps.pitch):
                state = STOPPED
            elif not rc.is_script_running:
                # Making sure the rc.is_script_running has time to update
                time.sleep(0.1)
                if not rc.is_script_running:
                    state = STOPPED

        gui.reset_button_clicks()
        state_publisher.publish( state )

        rate.sleep()
