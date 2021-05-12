#!/usr/bin/env python

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, String
import time

# Conversion dictionary for motor controller state
mc_state_description = {
    '0000000': "not ready to switch on",
    '1000000': "switch on disabled",
    '0100001': "ready to switch on",
    '0100011': "switched on",
    '0100111': "operation enabled",
    '0000111': "quick stop active",
    '0001111': "fault reaction active",
    '0001000': "fault",
}

# Conversion dictionary for CANopen NMT state
nmt_state_description = {
    0: "boot up",
    4: "stopped",
    5: "operational",
    127: "pre-operational",
}

class MotorControllerNode:
    def __init__(self, node, topic_name):
        self.gear_ratio = rospy.get_param('~gear_ratio')
        self.rated_current = rospy.get_param('~rated_current')
        self.node = node
        self.topic_name = topic_name

        # Change motor controller internal state
        ## Send a "shut down" command to go to "Ready to Switch On" state
        self.sdo_controlword = self.node.sdo[0x6040]
        self.sdo_controlword.raw = int('0110',2)
        time.sleep(0.01)

        ## Then send a "Enable Operation" command to go to the "Operation Enable" state
        ## This state enables power to motor
        self.sdo_controlword.raw = int('1111',2)
        time.sleep(0.01)

        # Change motor controller NMT state to Operational, which allows PDO communication
        self.node.nmt.state = "OPERATIONAL"
        time.sleep(0.01)

        # PDO setup
        self.node.tpdo.read()
        self.node.rpdo.read()

        # PDO callbacks
        self.node.tpdo[1].add_callback(self.tpdo1_cb)

        # Heartbeat callbacks
        self.node.nmt.add_hearbeat_callback(self.heartbeat_cb)

        # Publishers
        self.pub_state = rospy.Publisher('/motor_controller/{}/state'.format(self.topic_name), String, queue_size=10)
        self.pub_heartbeat = rospy.Publisher('/motor_controller/{}/heartbeat'.format(self.topic_name), String, queue_size=10)
        self.pub_motor_current = rospy.Publisher('/motor/{}/current_draw'.format(self.topic_name), Float32, queue_size=10)
        self.pub_wheel_rpm_actual = rospy.Publisher('/motor/{}/wheel_rpm_actual'.format(self.topic_name), Float32, queue_size=10)

    def spin(self, wheel_rpm):
        self.node.rpdo[1][0].raw = self.gear_ratio * wheel_rpm
        self.node.rpdo[1].transmit()
    
    def tpdo1_cb(self, tpdo1):
        # tpdo1 contains three element:
        # 0: statusword
        # 1: current, the value of 1000 means rated current is being drawn
        # 2: actual velociy of motor input shaft

        # 0
        state_binary_string = '{0:07b}'.format(tpdo1[0].raw) # convert int to binary string, padding it to 7 bits if needed
        state_binary_string = state_binary_string[-7:] # only the least significant 7 bits represent the state
        state_description = mc_state_description.get(state_binary_string)
        self.pub_state.publish( state_description )

        # 1
        self.pub_motor_current.publish(abs( tpdo1[1].raw / 1000 * self.rated_current) )

        # 2
        self.pub_wheel_rpm_actual.publish( tpdo1[2].raw / self.gear_ratio )

    def heartbeat_cb(self, nmt_state_int):
        self.pub_heartbeat.publish(nmt_state_description.get(nmt_state_int))

class MotorControllerNetwork:
    def __init__(self):
        # Get motor controller parameters
        mc_lf_node_id = rospy.get_param('~mc_lf_node_id') # motor controller at left front
        mc_lb_node_id = rospy.get_param('~mc_lb_node_id') # motor controller at left back
        mc_rf_node_id = rospy.get_param('~mc_rf_node_id') # motor controller at right front
        mc_rb_node_id = rospy.get_param('~mc_rb_node_id') # motor controller at right back
        local_node_id = rospy.get_param('~local_node_id')

        mc_eds_path = rospy.get_param('~mc_eds_path')
        local_eds_path = rospy.get_param('~local_eds_path')

        # Get CAN parameters
        channel = rospy.get_param('~channel')
        bustype = rospy.get_param('~bustype')
        bitrate = rospy.get_param('~bitrate')

        # Create CanOpen network
        self.network = canopen.Network()
        self.network.connect(channel=channel, bustype=bustype, bitrate=bitrate)

        # Create nodes
        node = self.network.add_node(mc_lf_node_id, mc_eds_path)
        self.mc_lf_node = MotorControllerNode(node, 'left_front')

        node = self.network.add_node(mc_rf_node_id, mc_eds_path)
        self.mc_rf_node = MotorControllerNode(node, 'right_front')
        
        self.local_node = canopen.LocalNode(local_node_id, local_eds_path)
        self.network.add_node(self.local_node)

        # Subscribers
        rospy.Subscriber('/wheel_rpm_command', WheelRPM, self.drive, queue_size=1)

    def drive(self, msg_wheel_rpm):
        self.mc_lf_node.spin(msg_wheel_rpm.left_front)
        self.mc_rf_node.spin(msg_wheel_rpm.right_front)
            
if __name__ ==  '__main__':
    node = rospy.init_node('can_interface', disable_signals=True)

    motor_controller_network = MotorControllerNetwork()

    rospy.spin()

    

