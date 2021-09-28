#!/usr/bin/env python

# Abbreviations
# mc: motor controller
# mcs: motor controllers (as a list)
# lf: left front
# lb: left back
# rf: right front
# rb: right back

import rospy
from std_msgs.msg import Int32, Empty, Bool
import time
import enum
import threading
from datetime import datetime

# States
MANUAL = 1
AUTO = 2
E_STOPPED = 3
STOPPED = 5

class ErrorHandler:
    def __init__(self, mcs, gui, handheld):
        self.mcs = mcs
        self.gui = gui
        self.handheld = handheld

        # Publishers
        self.error_publisher = rospy.Publisher('/overseer/has_error', Empty, queue_size=10)

    def has_error(self, overseer_state, should_log_error):
        errors = ""
        has_error = False

        # Motor controllers
        for mc in self.mcs:
            error_word = mc.error_word
            if error_word != 0:
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
            pass # implement this later (for GPU error) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Log errors and inform GUI
        if has_error and should_log_error:
            self.error_publisher.publish()

            time = datetime.now().strftime("%H:%M:%S")
            errors = time + "\n" + errors + "\n"

            date = datetime.now().strftime("%Y_%m_%d")
            filename = 'error_log_{}.txt'.format(date)
            with open('../../../error_log/' + filename, 'a') as f:
                f.write(errors)

        return has_error

class MotorController:
    def __init__(self, location, name):
        self.location = location
        self.name = name
        self.error_word = 0
        self.is_heartbeat_timeout = False

        rospy.Subscriber('/motor_controller/{}/error_word'.format(self.location), Int32, self.set_error_word)
        rospy.Subscriber('/motor_controller/{}/is_heartbeat_timeout'.format(self.location), Bool, self.set_is_heartbeat_timeout)

    def set_is_heartbeat_timeout(self, msg):
        self.is_heartbeat_timeout = msg.data

    def set_error_word(self, msg):
        self.error_word = msg.data

class Gui:
    def __init__(self):
        self.heartbeat_arrival_time = self.get_time_now_in_ms()
        self.is_stop_clicked = False
        self.is_enable_manual_clicked = False
        self.is_heartbeat_timeout = False
        self.is_start_following_clicked = False

        rospy.Subscriber('/gui/stop_clicked', Empty, self.stop_callback)
        rospy.Subscriber('/gui/enable_manual_clicked', Empty, self.enable_manual_callback)
        rospy.Subscriber('/gui/heartbeat', Empty, self.heartbeat_callback)
        rospy.Subscriber('/gui/start_path_following_clicked', Empty, self.start_following_callback)

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

class Handheld:
    def __init__(self):
        self.is_estop_pressed = False
        rospy.Subscriber('/handheld/is_estop_pressed', Bool, self.estop_callback)

    def estop_callback(self, msg):
        self.is_estop_pressed = msg.data

class RobotCommander:
    def __init__(self):
        self.is_script_running = False
        rospy.Subscriber('/robot_commander/is_script_running', Bool, self.is_script_running_callback)

    def is_script_running_callback(self, msg):
        self.is_script_running = msg.data

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
            elif gui.is_stop_clicked or error_handler.has_error(state, True):
                state = STOPPED
        
        # Auto
        elif state == AUTO:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_stop_clicked or error_handler.has_error(state, True):
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

        gui.reset_button_clicks()
        state_publisher.publish( state )

        rate.sleep()
