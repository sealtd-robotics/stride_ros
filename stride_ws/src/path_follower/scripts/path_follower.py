# Abbreviations
# v: linear velocity
# w: angular velocity

import rospy
from std_msgs.msg import Int32

class PathFollower:
    def __init__(self):
        self.overseer_state = 5    # 5: STOPPED state
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback, queue_size=10)

    def overseer_state_callback(self, new_state):
        self.overseer_state = new_state.data

    def find_velocities(self):
        v = 1
        w = 2
        return v, w

