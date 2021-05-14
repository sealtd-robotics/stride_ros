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

state_to_int = {
    'initial': 0,
    'manual': 1,
    'auto': 2,
    'e_stopped': 3,
    'fault': 4,
    'gui_stopped': 5,
}

class Overseer(Machine):
    def __init__(self):
        states = ['initial', 'manual', 'auto', 'e_stopped', 'fault', 'gui_stopped']
        Machine.__init__(self, states=states, initial='initial', auto_transitions=False)
        self.add_transition('initialize', 'initial', 'manual')
        self.add_transition('to_e_stopped', '*', 'e_stopped')
        self.add_transition('to_manual', ['e_stopped', 'fault', 'gui_stopped'] 'manual')
        self.add_transition('to_fault', ['manual', 'auto'], 'fault')
        # self.add_transition('recover', 'fault', 'manual')
        # self.add_transition('gui_start', 'gui_stopped', 'manual')
        self.add_transition('to_gui_stopped', ['manual', 'auto'], 'gui_stopped')

        # GUI
        self.gui = Gui()

        # Publishers
        self.state_publisher = rospy.Publisher('/overseer/state', Int32, queue_size=10, latch=True)

        self.initialize()

    def on_enter_manual(self):
        self.state_publisher.publish( state_to_int[self.state] )

    def on_enter_auto(self):
        self.state_publisher.publish( state_to_int[self.state] )

    def on_enter_e_stopped(self):
        self.state_publisher.publish( state_to_int[self.state] )

    def on_enter_fault(self):
        self.state_publisher.publish( state_to_int[self.state] )

    def on_enter_gui_stopped(self):
        self.state_publisher.publish( state_to_int[self.state] )


class MotorController:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.error_word = 0

        rospy.Subscriber('/motor_controller/{}/state'.format(self.topic_name), Int32, self.set_error_word, queue_size=10)

    def set_error_word(msg):
        self.error_word = msg.data

def are_mcs_bad(mcs): # mcs = motor controllers (plural)
    return (
        mcs[0].error_word or
        mcs[1].error_word or
        mcs[2].error_word or
        mcs[3].error_word
    )

class Gui:
    def __init__(self):
        self.is_motion_stop_clicked = False
        self.is_motion_start_clicked = False
        self.is_fault_recovery_clicked = False
        rospy.Subscriber('/is_motion_stop_clicked', Empty, self.motion_stop_callback, queue_size=20)
        rospy.Subscriber('/is_motion_start_clicked', Empty, self.motion_start_callback, queue_size=20)
        rospy.Subscriber('/is_fault_recovery_clicked', Empty, self.fault_recovery_callback, queue_size=20)

    def motion_stop_callback(self):
        self.is_motion_stop_clicked = True

    def motion_start_callback(self):
        self.is_motion_start_clicked = True

    def is_fault_recovery_callback(self):
        self.is_fault_recovery_clicked = True

class Handheld:
    def __init__(self):
        self.is_estop_pressed = False
        rospy.Subscriber('/is_estop_pressed', Bool, self.estop_callback, queue_size=20)

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

    # Overseer
    overseer = Overseer()

    while not rospy.is_shutdown():
        if overseer.state == 'manual':
            if handheld.is_estop_pressed:
                overseer.to_e_stopped()
            elif are_mcs_bad(mcs):
                overseer.to_fault()
            elif gui.is_motion_stop_clicked:
                gui.is_motion_stop_clicked = False
                overseer.to_gui_stopped()
        elif overseer.state == 'auto':
            if handheld.is_estop_pressed:
                overseer.to_e_stopped()
            elif are_mcs_bad(mcs) or 0: # add gps_bad condition here
                overseer.to_fault()
            elif gui.is_motion_stop_clicked:
                gui.is_motion_stop_clicked = False
                overseer.to_gui_stopped()
        elif overseer.state == 'e_stopped':
            if not handheld.is_estop_pressed:
                if are_mcs_bad(mcs):
                    overseer.to_fault()
                else:
                    overseer.to_manual()
        elif overseer.state == 'fault':
            if handheld.is_estop_pressed:
                overseer.to_e_stopped()
            elif gui.is_fault_recovery_clicked
                gui.is_fault_recovery_clicked = False
                if not are_mcs_bad(mcs):
                    overseer.to_manual()
        elif overseer.state == 'gui_stopped':
            if gui.is_motion_start_clicked:
                gui.is_motion_start_clicked = False
                if not are_mcs_bad(mcs):
                    overseer.to_manual()
            elif handheld.is_estop_pressed:
                overseer.to_e_stopped()
            


    rospy.spin()
