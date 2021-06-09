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

    def has_error(self, overseer_state, log_error):
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
        if overseer_state == AUTO: # implement this later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            pass

        # Log errors and inform GUI
        if has_error and log_error:
            self.error_publisher.publish()

            time = datetime.now().strftime("%H:%M:%S")
            errors = time + "\n" + errors + "\n"

            date = datetime.now().strftime("%Y_%m_%d")
            filename = 'error_log_{}.txt'.format(date)
            with open('../../../error_log/' + filename, 'a') as f:
                f.write(errors)

        return has_error

class MotorController:
    def __init__(self, topic_name, name):
        self.topic_name = topic_name
        self.name = name
        self.error_word = 0
        self.state = 0
        self.nmt_state = 0
        self.is_heartbeat_timeout = False

        rospy.Subscriber('/motor_controller/{}/error_word'.format(self.topic_name), Int32, self.set_error_word)
        rospy.Subscriber('/motor_controller/{}/state'.format(self.topic_name), Int32, self.set_state)
        rospy.Subscriber('/motor_controller/{}/heartbeat_nmt'.format(self.topic_name), Int32, self.set_nmt_state)
        rospy.Subscriber('/motor_controller/{}/is_heartbeat_timeout'.format(self.topic_name), Bool, self.set_is_heartbeat_timeout)

    def set_is_heartbeat_timeout(self, msg):
        self.is_heartbeat_timeout = msg.data

    def set_error_word(self, msg):
        self.error_word = msg.data

    def set_state(self, msg):
        self.state = msg.data

    def set_nmt_state(self, msg):
        self.nmt_state = msg.data

class Gui:
    def __init__(self):
        self.is_stop_clicked = False
        self.is_enable_manual_clicked = False
        self.heartbeat_arrival_time = self.get_time_now_in_ms()
        self.is_heartbeat_timeout = False

        rospy.Subscriber('/gui/stop_clicked', Empty, self.stop_callback)
        rospy.Subscriber('/gui/enable_manual_clicked', Empty, self.enable_manual_callback)
        rospy.Subscriber('/gui/heartbeat', Empty, self.heartbeat_callback)

        # Heartbeat timeout thread
        self.heartbeat_thread = threading.Thread(target=self.monitor_heartbeat)
        self.heartbeat_thread.start()

    def monitor_heartbeat(self):
        while True:
            if self.get_time_now_in_ms() - self.heartbeat_arrival_time > 1000:
                self.is_heartbeat_timeout = True
            else:
                self.is_heartbeat_timeout = False
            time.sleep(0.1)


    def reset_button_clicks(self): # this is for handling burst of clicks in a slow network and hanlding buffered clicks not consumed by current state
        self.is_stop_clicked = False
        self.is_enable_manual_clicked = False

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

class Handheld:
    def __init__(self):
        self.is_estop_pressed = False
        rospy.Subscriber('/handheld/is_estop_pressed', Bool, self.estop_callback)

    def estop_callback(self, msg):
        self.is_estop_pressed = msg.data

if __name__ ==  '__main__':
    node = rospy.init_node('overseer')

    # Motor Controllers
    mc_lf = MotorController('left_front', 'Motor Controller 1')
    mc_lb = MotorController('left_back', 'Motor Controller 2')
    mc_rf = MotorController('right_front', 'Motor Controller 3')
    mc_rb = MotorController('right_back', 'Motor Controller 4')
    mcs = [mc_lf, mc_lb, mc_rf, mc_rb] # mcs = motor controllers

    gui = Gui()

    handheld = Handheld()

    error_handler = ErrorHandler(mcs, gui, handheld)

    # initial state
    state = STOPPED

    # Publishers
    state_publisher = rospy.Publisher('/overseer/state', Int32, queue_size=10, latch=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Manual
        if state == MANUAL:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_stop_clicked or error_handler.has_error(state, True):
                state = STOPPED
            gui.reset_button_clicks()
        
        # Auto (currently can't go to this state)
        elif state == AUTO:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_stop_clicked or error_handler.has_error(state, True):
                state = STOPPED
            gui.reset_button_clicks()

        # E_Stopped
        elif state == E_STOPPED:
            if not handheld.is_estop_pressed:
                state = STOPPED
            gui.reset_button_clicks()

        # Stopped
        elif state == STOPPED:
            if gui.is_enable_manual_clicked and not error_handler.has_error(state, False):
                state = MANUAL
            elif handheld.is_estop_pressed:
                state = E_STOPPED

            # todo: add elif for going to AUTO

            gui.reset_button_clicks()

        state_publisher.publish( state )
        rate.sleep()
