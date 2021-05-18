#!/usr/bin/env python

# Abbreviations
# mc: motor controller
# mcs: motor controllers (plural)
# lf: left front
# lb: left back
# rf: right front
# rb: right back

import rospy
from transitions import Machine
from std_msgs.msg import Int32, Empty, Bool
import time
import enum

# States
MANUAL = 1
AUTO = 2
E_STOPPED = 3
ERROR = 4
STOPPED = 5

class MotorController:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.error_word = 0
        self.state = 0
        self.nmt_state = 0

        rospy.Subscriber('/motor_controller/{}/error_word'.format(self.topic_name), Int32, self.set_error_word)
        rospy.Subscriber('/motor_controller/{}/state'.format(self.topic_name), Int32, self.set_state)
        rospy.Subscriber('/motor_controller/{}/heartbeat_nmt'.format(self.topic_name), Int32, self.set_nmt_state)

    def set_error_word(self, msg):
        self.error_word = msg.data

    def set_state(self, msg):
        self.state = msg.data

    def set_nmt_state(self, msg):
        self.nmt_state = msg.data

def are_mcs_bad(mcs): # mcs = motor controllers (plural)
    are_bad = False

    # state: 39 is operation enabled
    # nmt_state: 5 is operational
    for mc in mcs:
        are_bad =  are_bad or mc.error_word #or mc.state != 39 or mc.nmt_state != 5
    
    return are_bad


class Gui:
    def __init__(self):
        self.is_stop_clicked = False
        self.is_start_clicked = False
        self.is_recover_clicked = False
        rospy.Subscriber('/gui/stop_clicked', Empty, self.stop_callback)
        rospy.Subscriber('/gui/start_clicked', Empty, self.start_callback)
        rospy.Subscriber('/gui/recover_clicked', Empty, self.recover_callback)

    def stop_callback(self, msg):
        self.is_stop_clicked = True

    def start_callback(self, msg):
        self.is_start_clicked = True

    def recover_callback(self, msg):
        self.is_recover_clicked = True

class Handheld:
    def __init__(self):
        self.is_estop_pressed = False
        rospy.Subscriber('/handheld/is_estop_pressed', Bool, self.estop_callback)

    def estop_callback(self, msg):
        self.is_estop_pressed = msg.data

if __name__ ==  '__main__':
    node = rospy.init_node('overseer')

    # Motor Controllers
    mc_lf = MotorController('left_front')
    mc_lb = MotorController('left_back')
    mc_rf = MotorController('right_front')
    mc_rb = MotorController('right_back')

    mcs = [mc_lf, mc_lb, mc_rf, mc_rb] # mcs = motor controllers (plural)

    # GUI
    gui = Gui()

    # Handheld
    handheld = Handheld()

    # Publishers
    state_publisher = rospy.Publisher('/overseer/state', Int32, queue_size=10, latch=True)
    
    # Set initial state
    state = MANUAL

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state == MANUAL:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif are_mcs_bad(mcs):
                state = ERROR
            elif gui.is_stop_clicked:
                gui.is_stop_clicked = False
                state = STOPPED
        elif state == AUTO:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif are_mcs_bad(mcs) or 0: # add gps_bad condition here
                state = ERROR
            elif gui.is_stop_clicked:
                gui.is_stop_clicked = False
                state = STOPPED
        elif state == E_STOPPED:
            if not handheld.is_estop_pressed:
                if are_mcs_bad(mcs):
                    state = ERROR
                else:
                    state = MANUAL
        elif state == ERROR:
            if handheld.is_estop_pressed:
                state = E_STOPPED
            elif gui.is_recover_clicked:
                gui.is_recover_clicked = False
                if not are_mcs_bad(mcs):
                    state = MANUAL
        elif state == STOPPED:
            if gui.is_start_clicked:
                gui.is_start_clicked = False
                if are_mcs_bad(mcs):
                    state = ERROR
                else:
                    state = MANUAL
            elif handheld.is_estop_pressed:
                state = E_STOPPED

        state_publisher.publish( state )
        rate.sleep()
