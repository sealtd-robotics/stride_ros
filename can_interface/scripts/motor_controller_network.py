#!/usr/bin/env python

# Abbreviations
# mc: motor controler
# lf: left front
# lb: left back
# rf: right front
# rb: right back

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, Int32
import time

class MotorControllerNode:
    def __init__(self, node, topic_name):
        self.gear_ratio = rospy.get_param('~gear_ratio')
        self.rated_current = rospy.get_param('~rated_current')

        self.node = node
        self.topic_name = topic_name

        self.overseer_state = 0
        self.sdo_controlword = self.node.sdo[0x6040]

        # Change motor controller internal state
        self.enable_power()

        # Change motor controller NMT state to Operational, which allows PDO communication
        self.change_nmt_state('OPERATIONAL')

        # PDO setup
        self.node.tpdo.read()
        self.node.rpdo.read()

        # PDO callbacks
        self.node.tpdo[1].add_callback(self.tpdo1_callback)
        self.node.tpdo[2].add_callback(self.tpdo2_callback)

        # Heartbeat callbacks
        self.node.nmt.add_hearbeat_callback(self.heartbeat_callback)

        # Publishers
        self.state_publisher = rospy.Publisher('/motor_controller/{}/state'.format(self.topic_name), Int32, queue_size=10)
        self.heartbeat_publisher = rospy.Publisher('/motor_controller/{}/heartbeat_nmt'.format(self.topic_name), Int32, queue_size=10)
        self.motor_current_publisher = rospy.Publisher('/motor_controller/{}/motor_current_draw'.format(self.topic_name), Float32, queue_size=10)
        self.wheel_rpm_actual_publisher = rospy.Publisher('/motor_controller/{}/wheel_rpm_actual'.format(self.topic_name), Float32, queue_size=10)
        self.error_word_publisher = rospy.Publisher('/motor_controller/{}/error_word'.format(self.topic_name), Int32, queue_size=10)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

    def overseer_state_callback(self, new_state):
        if self.overseer_state == new_state.data:
            return

        is_current_state_inoperable = self.overseer_state == 3 or \
                                        self.overseer_state == 4 or \
                                        self.overseer_state == 5

        # if overseer state changes from inoperable to manual, run recovery steps
        if is_current_state_inoperable and new_state.data == 1:
            self.fault_reset_command()
            self.enable_power()
            self.change_nmt_state('OPERATIONAL')

        self.overseer_state = new_state.data

    def change_nmt_state(self, nmt_state):
        # nmt_state is a string
        self.node.nmt.state = nmt_state
        time.sleep(0.02)

    def fault_reset_command(self):
        self.sdo_controlword.raw = int('10000000',2) # 1000 0000
        time.sleep(0.02)

    def enable_power(self):
        # Send a "shut down" command to go to "Ready to Switch On" state
        self.sdo_controlword.raw = int('0110',2)
        time.sleep(0.02)

        # Then send a "Enable Operation" command to go to the "Operation Enable" state
        # This state enables power to motor
        self.sdo_controlword.raw = int('1111',2)
        time.sleep(0.02)

    def spin(self, wheel_rpm):
        self.node.rpdo[1][0].raw = self.gear_ratio * wheel_rpm
        self.node.rpdo[1].transmit()
    
    def tpdo1_callback(self, tpdo1):
        # tpdo1 contains the following elements:
        # 0: statusword
        # 1: current, the value of 1000 means rated current is being drawn
        # 2: actual velociy of motor input shaft

        # 0
        self.state_publisher.publish( tpdo1[0].raw )

        # 1
        self.motor_current_publisher.publish(abs( tpdo1[1].raw / 1000 * self.rated_current) )

        # 2
        self.wheel_rpm_actual_publisher.publish( tpdo1[2].raw / self.gear_ratio )

    def tpdo2_callback(self, tpdo2):
        # tpdo2 contains the following elements:
        # 0: error_word

        # 0
        self.error_word_publisher.publish(tpdo2[0].raw)

    def heartbeat_callback(self, nmt_state_int):
        self.heartbeat_publisher.publish(nmt_state_int)

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
        # node = self.network.add_node(mc_lf_node_id, mc_eds_path)
        # self.mc_lf_node = MotorControllerNode(node, 'left_front')

        node = self.network.add_node(mc_rf_node_id, mc_eds_path)
        self.mc_rf_node = MotorControllerNode(node, 'right_front')
        
        self.local_node = canopen.LocalNode(local_node_id, local_eds_path)
        self.network.add_node(self.local_node)

        # Subscribers
        rospy.Subscriber('/wheel_rpm_command', WheelRPM, self.drive, queue_size=1)

    def drive(self, msg_wheel_rpm):
        # self.mc_lf_node.spin(msg_wheel_rpm.left_front)
        self.mc_rf_node.spin(msg_wheel_rpm.right_front)
            
if __name__ ==  '__main__':
    node = rospy.init_node('can_interface')

    motor_controller_network = MotorControllerNetwork()

    rospy.spin()

    

