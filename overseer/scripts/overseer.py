#!/usr/bin/env python

import rospy
from transitions import Machine
from std_msgs.msg import Int32, String
import time

class Overseer(Machine):
    def __init__(self):
        states = ['initial', 'fault', 'stopped', 'manual', 'auto']
        Machine.__init__(self, states=states, initial='initial', auto_transitions=False)
        self.add_transition('initialize', 'initial', 'manual')
        self.add_transition('stop', '*', 'stopped')
        self.add_transition('resume', 'stopped', 'manual')
        self.add_transition('crash', ['manual', 'auto'], 'fault')
        self.add_transition('recover', 'fault', 'manual')

        # Publishers
        self.state_publisher = rospy.Publisher('/overseer/state', String, queue_size=10, latch=True)

        self.initialize()

    def on_enter_manual(self):
        print(self.state)
        self.state_publisher.publish(self.state)

if __name__ ==  '__main__':
    node = rospy.init_node('overseer')

    overseer = Overseer()

    rospy.spin()
