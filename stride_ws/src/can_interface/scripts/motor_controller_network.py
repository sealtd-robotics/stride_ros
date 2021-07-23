#!/usr/bin/env python

# Abbreviations
# mcn: MotorControllerNetwork
# mc: motor controller
# lf: left front
# lb: left back
# rf: right front
# rb: right back

from __future__ import division
import canopen
import rospy
from can_interface.msg import WheelRPM
from std_msgs.msg import Float32, Int32, Bool, Empty
import time
import threading
from datetime import datetime

class MotorControllerNode:
    def __init__(self, node, topic_name):
        self.gear_ratio = rospy.get_param('~gear_ratio')
        self.rated_current = rospy.get_param('~rated_current')
        self.error_word = 0
        self.state = 0
        self.wheel_rpm_actual = 0

        self.node = node
        self.topic_name = topic_name

        self.heartbeat_arrival_time = self.get_time_now_in_ms() + 5000 # added some time buffer for startup
        self.overseer_state = 0
        self.sdo_controlword = self.node.sdo[0x6040]

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
        self.heartbeat_timeout_publisher = rospy.Publisher('/motor_controller/{}/is_heartbeat_timeout'.format(self.topic_name), Bool, queue_size=10)
        self.motor_current_publisher = rospy.Publisher('/motor_controller/{}/motor_current_draw'.format(self.topic_name), Float32, queue_size=10)
        self.wheel_rpm_actual_publisher = rospy.Publisher('/motor_controller/{}/wheel_rpm_actual'.format(self.topic_name), Float32, queue_size=10)
        self.error_word_publisher = rospy.Publisher('/motor_controller/{}/error_word'.format(self.topic_name), Int32, queue_size=10)

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

        # Heartbeat timeout thread
        self.heartbeat_thread = threading.Thread(target=self.monitor_heartbeat)
        self.heartbeat_thread.setDaemon(True)
        self.heartbeat_thread.start()

    def monitor_heartbeat(self):
        while True:
            if self.get_time_now_in_ms() - self.heartbeat_arrival_time > 1000:
                self.heartbeat_timeout_publisher.publish(True)
            else:
                self.heartbeat_timeout_publisher.publish(False)
            time.sleep(0.1)

    # May not be needed anymore....... Test it !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    def overseer_state_callback(self, new_state):
        if self.overseer_state == new_state.data:
            return

        is_current_state_inoperable = self.overseer_state == 3 or \
                                        self.overseer_state == 4 or \
                                        self.overseer_state == 5

        # if overseer state changes from inoperable to manual or auto, run recovery steps
        if is_current_state_inoperable and (new_state.data == 1 or new_state.data == 2):
            self.fault_reset_command()
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

    def quick_stop_controlword(self):
        self.sdo_controlword.raw = 2
        time.sleep(0.02)

    def spin(self, wheel_rpm):
        self.node.rpdo[1][0].raw = self.gear_ratio * wheel_rpm
        self.node.rpdo[1].transmit()
    
    def tpdo1_callback(self, tpdo1):
        # tpdo1 contains the following elements:
        # 0: statusword
        # 1: current, the value of 1000 means rated current is being drawn
        # 2: actual velocity of motor input shaft

        # 0
        # only the first 7 bits represent the state. "0111 1111" is 127
        state = tpdo1[0].raw & 127
        self.state_publisher.publish(state)
        self.state = state

        # 1
        self.motor_current_publisher.publish(abs( tpdo1[1].raw / 1000 * self.rated_current) )

        # 2
        self.wheel_rpm_actual_publisher.publish( tpdo1[2].raw / self.gear_ratio )
        self.wheel_rpm_actual = tpdo1[2].raw / self.gear_ratio

    def tpdo2_callback(self, tpdo2):
        # tpdo2 contains the following elements:
        # 0: error_word

        # 0
        error_word = tpdo2[0].raw
        self.error_word_publisher.publish(error_word)
        self.error_word = error_word

    def get_time_now_in_ms(self):
        epoch = datetime.utcfromtimestamp(0)
        now = datetime.utcnow()
        delta = now - epoch
        return delta.total_seconds() * 1000

    def heartbeat_callback(self, nmt_state_int):
        self.heartbeat_publisher.publish(nmt_state_int)

        if nmt_state_int == 127: # "pre-operational":
            self.change_nmt_state('OPERATIONAL')

        self.heartbeat_arrival_time = self.get_time_now_in_ms()

class MotorControllerNetwork:
    def __init__(self):
        self.overseer_state = 0
        self.does_brake_when_stopped = False
        self.has_quick_stop_been_applied = False

        self.left_front_rpm = 0
        self.left_back_rpm = 0
        self.right_front_rpm = 0
        self.right_back_rpm = 0

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

        # Create local node
        self.local_node = canopen.LocalNode(local_node_id, local_eds_path)
        self.network.add_node(self.local_node)

        ## start heartbeat
        self.local_node.nmt.start_heartbeat(300)

        # Create motor controller nodes
        node = self.network.add_node(mc_lf_node_id, mc_eds_path)
        self.mc_lf_node = MotorControllerNode(node, 'left_front')

        node = self.network.add_node(mc_lb_node_id, mc_eds_path)
        self.mc_lb_node = MotorControllerNode(node, 'left_back')

        node = self.network.add_node(mc_rf_node_id, mc_eds_path)
        self.mc_rf_node = MotorControllerNode(node, 'right_front')

        node = self.network.add_node(mc_rb_node_id, mc_eds_path)
        self.mc_rb_node = MotorControllerNode(node, 'right_back')

        # Publishers
        self.does_brake_when_stopped_publisher = rospy.Publisher('/motor_controller_network/does_brake_when_stopped', Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber('/wheel_rpm_command', WheelRPM, self.set_rpm, queue_size=1)
        rospy.Subscriber('/overseer/state', Int32, self.set_overseer_state)
        rospy.Subscriber('/gui/brake_when_stopped_toggled', Empty, self.toggle_does_brake_when_stopped)

        # Thread to continuously publish brake_when_stopped boolean
        self.mcn_thread_1 = threading.Thread(target=self.publish_does_brake_when_stopped)
        self.mcn_thread_1.setDaemon(True)
        self.mcn_thread_1.start()

        # Thread to continously send out motion-related CAN commands to all motors
        self.mcn_thread_2 = threading.Thread(target=self.drive)
        self.mcn_thread_2.setDaemon(True)
        self.mcn_thread_2.start()

        # Thread to relax motor when braked to prevent continuous high current draw
        self.mcn_thread_3 = threading.Thread(target=self.relax_motors)
        self.mcn_thread_3.setDaemon(True)
        self.mcn_thread_3.start()

    def can_relax(self):
        return self.overseer_state == 1 and self.does_brake_when_stopped and \
                self.left_front_rpm == 0 and self.left_back_rpm == 0 and self.right_front_rpm == 0 and self.right_back_rpm == 0

    def relax_motors(self):
        interval = 5
        while True:
            time.sleep(interval)
            if self.can_relax():
                # enable_power() actually disables and then re-enables power 
                self.mc_lf_node.enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_lb_node.enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_rf_node.enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_rb_node.enable_power()

    def set_overseer_state(self, msg):
        self.overseer_state = msg.data

    def toggle_does_brake_when_stopped(self, msg):
        self.does_brake_when_stopped = not self.does_brake_when_stopped

    def publish_does_brake_when_stopped(self):
        while True:
            if self.overseer_state == 5:
                self.does_brake_when_stopped = False
            self.does_brake_when_stopped_publisher.publish(self.does_brake_when_stopped)
            time.sleep(0.1)

    def set_rpm(self,msg):
        self.left_front_rpm = msg.left_front
        self.left_back_rpm = msg.left_back
        self.right_front_rpm = msg.right_front
        self.right_back_rpm = msg.right_back

    def enable_power_for_all_motors(self):
        # this condition is to prevent the error of enabling power when the main battery is unplugged
        if self.has_quick_stop_been_applied:
            self.mc_lf_node.enable_power()
            self.mc_lb_node.enable_power()
            self.mc_rf_node.enable_power()
            self.mc_rb_node.enable_power()

            self.has_quick_stop_been_applied = False

    def send_zero_rpm_to_all_motors(self):
        self.mc_lf_node.spin(0)
        self.mc_lb_node.spin(0)
        self.mc_rf_node.spin(0)
        self.mc_rb_node.spin(0)

    def quick_stop_all_motors(self):
        self.mc_lf_node.quick_stop_controlword()
        self.mc_lb_node.quick_stop_controlword()
        self.mc_rf_node.quick_stop_controlword()
        self.mc_rb_node.quick_stop_controlword()

        self.has_quick_stop_been_applied = True

    def is_any_measured_wheel_rpm_above_this(self, rpm):
        return (
            abs(self.mc_lb_node.wheel_rpm_actual) > rpm or
            abs(self.mc_lf_node.wheel_rpm_actual) > rpm or
            abs(self.mc_rf_node.wheel_rpm_actual) > rpm or
            abs(self.mc_rb_node.wheel_rpm_actual) > rpm
        )

    def drive(self): # type of msg is WheelRPM
        while True:
            time.sleep(0.1)
            if self.overseer_state == 5: # STOPPED state
                while self.is_any_measured_wheel_rpm_above_this(450):
                    self.enable_power_for_all_motors()
                    self.send_zero_rpm_to_all_motors()
                self.quick_stop_all_motors()
            elif self.overseer_state == 1: # MANUAL state
                if self.left_front_rpm == 0 and self.left_back_rpm == 0 and self.right_front_rpm == 0 and self.right_back_rpm == 0:
                    if self.does_brake_when_stopped:
                        self.enable_power_for_all_motors()
                        self.send_zero_rpm_to_all_motors()
                    else:
                        while self.is_any_measured_wheel_rpm_above_this(450):
                            self.enable_power_for_all_motors()
                            self.send_zero_rpm_to_all_motors()
                        self.quick_stop_all_motors()
                else:
                    self.enable_power_for_all_motors()

                    self.mc_lf_node.spin(self.left_front_rpm)
                    self.mc_lb_node.spin(self.left_back_rpm)
                    self.mc_rf_node.spin(self.right_front_rpm)
                    self.mc_rb_node.spin(self.right_back_rpm)

if __name__ ==  '__main__':
    node = rospy.init_node('can_interface')

    motor_controller_network = MotorControllerNetwork()
    
    rospy.spin()
        


    

