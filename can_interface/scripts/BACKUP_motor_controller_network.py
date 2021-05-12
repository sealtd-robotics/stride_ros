#!/usr/bin/env python

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, String
import time

class MotorControllerNetwork:
    def __init__(self):
        # Get motor parameters
        self.gear_ratio = rospy.get_param('~gear_ratio')
        self.rated_current = rospy.get_param('~rated_current')
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
        self.mc_lf_node = self.network.add_node(mc_lf_node_id, mc_eds_path)
        
        self.local_node = canopen.LocalNode(local_node_id, local_eds_path)
        self.network.add_node(self.local_node)

        # Change motor controller internal state
        ## Send a "shut down" command to go to "Ready to Switch On" state
        self.sdo_controlword_mc_lf = self.mc_lf_node.sdo[0x6040]
        self.sdo_controlword_mc_lf.raw = int('0110',2)
        time.sleep(0.01)

        ## Then send a "Enable Operation" command to go to the "Operation Enable" state
        ## This state enables power to motor
        self.sdo_controlword_mc_lf.raw = int('1111',2)
        time.sleep(0.01)

        # Change motor controller NMT state to Operational, which allows PDO communication
        self.mc_lf_node.nmt.state = "OPERATIONAL"
        time.sleep(0.01)

        # PDO setup
        self.mc_lf_node.tpdo.read()
        self.mc_lf_node.rpdo.read()

        # PDO callbacks
        self.mc_lf_node.tpdo[1].add_callback(self.mc_lf_tpdo1_cb)

        # Heartbeat callbacks
        self.mc_lf_node.nmt.add_hearbeat_callback(self.mc_lf_heartbeat_cb)

        # Publishers
        ## left front motor controller
        self.pub_mc_lf_state = rospy.Publisher('/motor_controller/left_front/state', String, queue_size=1)
        self.pub_mc_lf_heartbeat = rospy.Publisher('/motor_controller/left_front/heartbeat', String, queue_size=10)
        self.pub_motor_lf_current = rospy.Publisher('/motor/left_front/current_draw', Float32, queue_size=10)
        self.pub_motor_lf_wheel_rpm_actual = rospy.Publisher('/motor/left_front/wheel_rpm_actual', Float32, queue_size=10)

        # Subscribers
        rospy.Subscriber('/wheel_rpm_command', WheelRPM, self.drive)

        # Conversion dictionary for motor controller state
        self.mc_state_description = {
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
        self.nmt_state_description = {
            0: "boot up",
            4: "stopped",
            5: "operational",
            127: "pre-operational",
        }

    def mc_lf_tpdo1_cb(self, tpdo1):
        # tpdo1 contains three element:
        # 0: statusword
        # 1: current, the value of 1000 means rated current is being drawn
        # 2: actual velociy of motor input shaft

        # 0
        state_binary_string = '{0:07b}'.format(tpdo1[0].raw) # convert int to binary string, padding it to 7 bits if needed
        state_binary_string = state_binary_string[-7:] # only the least significant 7 bits represent the state
        state_description = self.mc_state_description.get(state_binary_string)
        self.pub_mc_lf_state.publish( state_description )

        # 1
        self.pub_motor_lf_current.publish(abs( tpdo1[1].raw / 1000 * self.rated_current) )

        # 2
        self.pub_motor_lf_wheel_rpm_actual.publish( tpdo1[2].raw / self.gear_ratio )

    def mc_lf_heartbeat_cb(self, nmt_state_int):
        self.pub_mc_lf_heartbeat.publish(self.nmt_state_description.get(nmt_state_int))

    def drive(self, msg_wheel_rpm):
        self.mc_lf_node.rpdo[1][0].raw = self.gear_ratio * msg_wheel_rpm.left_front
        self.mc_lf_node.rpdo[1].transmit()
            
if __name__ ==  '__main__':
    node = rospy.init_node('can_interface', disable_signals=True)

    motor_controller_network = MotorControllerNetwork()

    rospy.spin()

    

