#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

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
from shared_tools.overseer_states_constants import *

TIME_INTERVAL = 0.01

class MotorControllerNode:
    def __init__(self, node, topic_name):
        self.gear_ratio = rospy.get_param('gear_ratio')
        self.rated_current = rospy.get_param('rated_current')
        self.peak_current = rospy.get_param('peak_current')
        self.error_word = 0
        self.winding_temperature = 0
        self.state = 0
        self.wheel_rpm_actual = 0
        self.current = 0

        self.node = node
        self.topic_name = topic_name

        self.heartbeat_arrival_time = self.get_time_now_in_ms() + 5000 # added some time buffer for startup
        self.overseer_state = 0
        self.sdo_ambient_temperature = self.node.sdo[0x232A][0x08]
        self.sdo_peak_current = self.node.sdo[0x2329][0x03]

        # Set peak current
        self.sdo_peak_current.raw = self.peak_current

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
        self.winding_temperature_publisher = rospy.Publisher('/motor_controller/{}/winding_temperature'.format(self.topic_name), Int32, queue_size=10)

        # Heartbeat timeout thread
        self.heartbeat_thread = threading.Thread(target=self.monitor_heartbeat)
        self.heartbeat_thread.setDaemon(True)
        self.heartbeat_thread.start()

    def transmit_ambient_temperature(self, degree_F):
        degree_C = (degree_F - 32) * 5 / 9
        self.sdo_ambient_temperature.raw = degree_C

        time.sleep(TIME_INTERVAL)

    def monitor_heartbeat(self):
        while True:
            if self.get_time_now_in_ms() - self.heartbeat_arrival_time > 1000:
                self.heartbeat_timeout_publisher.publish(True)
            else:
                self.heartbeat_timeout_publisher.publish(False)
            time.sleep(0.1)

    def change_nmt_state(self, nmt_state):
        # nmt_state is a string
        self.node.nmt.state = nmt_state
        time.sleep(TIME_INTERVAL)

    def disable_enable_power(self):
        # disable and then re-enable power
        # can also be used for simply enabling power

        # Send a "shut down" command to go to "Ready to Switch On" state, from either "Operation Enabled" or "Switch on Disabled"
        self.node.rpdo[2][0].raw = int('0110',2)
        self.node.rpdo[2].transmit()
        time.sleep(TIME_INTERVAL)

        # Then send a "Enable Operation" command to go to the "Operation Enable" state
        # This state enables power to motor
        self.node.rpdo[2][0].raw = int('1111',2)
        self.node.rpdo[2].transmit()
        time.sleep(TIME_INTERVAL)

    def enable_power_if_disabled(self):
        # return when there is power already
        if self.state == 39 or self.state == 55: # Operation Enabled can be 39 or 55 
            return
        
        # The commands inside this function are same for enabling power
        self.disable_enable_power()

    def quick_stop_controlword(self):
        self.node.rpdo[2][0].raw = int('10',2)
        self.node.rpdo[2].transmit()
        time.sleep(TIME_INTERVAL)

    def spin(self, wheel_rpm):
        self.node.rpdo[1][0].raw = self.gear_ratio * wheel_rpm
        self.node.rpdo[1].transmit()
    
    def tpdo1_callback(self, tpdo1):
        # tpdo1 contains the following elements:
        # 0: statusword
        # 1: current, the value of 1000 means rated current is being drawn
        # 2: actual velocity of motor input shaft

        # 0
        # only the first 7 bits represent the state. "0111 1111" isleft_front_rpm 127
        state = tpdo1[0].raw & 127
        self.state_publisher.publish(state)
        self.state = state

        # 1
        self.current = abs( tpdo1[1].raw / 1000 * self.rated_current )
        self.motor_current_publisher.publish(self.current)

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

        # 1
        winding_temperature = tpdo2[1].raw
        winding_temperature = (winding_temperature * (9/5)) + 32
        self.winding_temperature_publisher.publish(winding_temperature)
        self.winding_temperature = winding_temperature

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
        self.ambient_temperature_F = 72
        self.disable_motor = False

        self.left_front_rpm = 0
        self.left_back_rpm = 0
        self.right_front_rpm = 0
        self.right_back_rpm = 0

        self.brake_command = 0
        self.brake_status = 3
        self.previous_state = -1
        # Get motor controller parameters
        mc_lf_node_id = rospy.get_param('mc_lf_node_id') # motor controller at left front
        mc_lb_node_id = rospy.get_param('mc_lb_node_id') # motor controller at left back
        mc_rf_node_id = rospy.get_param('mc_rf_node_id') # motor controller at right front
        mc_rb_node_id = rospy.get_param('mc_rb_node_id') # motor controller at right back
        local_node_id = rospy.get_param('local_node_id')

        mc_eds_path = rospy.get_param('mc_eds_path')
        local_eds_path = rospy.get_param('local_eds_path')

        # Get CAN parameters
        channel = rospy.get_param('channel')
        bustype = rospy.get_param('bustype')
        bitrate = rospy.get_param('bitrate')

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
        rospy.Subscriber('/robot_temperature', Int32, self.set_ambient_temperature, queue_size=1)
        rospy.Subscriber('/robot_commander/disable_motor', Bool, self.set_disable_motor, queue_size=1)
        rospy.Subscriber('/brake_status', Int32, self.brake_status_callback, queue_size=1)
        rospy.Subscriber('/brake_command', Bool, self.brake_command_callback, queue_size=1)

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

        # Thread to update ambient temperature of all motors
        self.mcn_thread_4 = threading.Thread(target=self.update_ambient_temperature)
        self.mcn_thread_4.setDaemon(True)
        self.mcn_thread_4.start()

    def can_relax(self):
        return (self.overseer_state == MANUAL or self.overseer_state == E_STOPPED or self.overseer_state == STOPPED) and \
                self.are_all_measured_wheel_rpm_below_this(10)

    def relax_motors(self):
        nodes = [self.mc_lf_node, self.mc_lb_node, self.mc_rf_node, self.mc_rb_node]
        # nodes = [self.mc_lf_node, self.mc_lb_node]
        interval = 3

        # relax the motor that draws the most current
        while True:
            try:
                time.sleep(interval)
                if self.can_relax():
                    max_current_node = nodes[0]
                    for i in range(1,4):
                        if max_current_node.current < nodes[i].current:
                            max_current_node = nodes[i]
                    max_current_node.disable_enable_power()
            except Exception as error:
                rospy.logerr("Ignore this error when power-cycling motor controllers. The relax_motors function from a thread of motor_controller_network.py raised an error, which says %s", error)
                time.sleep(1)
                continue
    
    def relax_motors_while_braking(self):
        nodes = [self.mc_lf_node, self.mc_lb_node, self.mc_rf_node, self.mc_rb_node]
        # nodes = [self.mc_lf_node, self.mc_lb_node]
        interval = 0.25

        # relax the motor that draws the most current
        while self.brake_status !=2 and self.overseer_state == AUTO:
            time.sleep(interval)
            max_current_node = nodes[0]
            for i in range(1,4):
                if max_current_node.current < nodes[i].current:
                    max_current_node = nodes[i]
            max_current_node.disable_enable_power()

    def set_overseer_state(self, msg):
        self.overseer_state = msg.data

    def toggle_does_brake_when_stopped(self, msg):
        self.does_brake_when_stopped = not self.does_brake_when_stopped

    def publish_does_brake_when_stopped(self):
        while True:
            self.does_brake_when_stopped_publisher.publish(self.does_brake_when_stopped)
            time.sleep(0.1)
    
    def set_ambient_temperature(self, msg):
        self.ambient_temperature_F = msg.data

    def set_disable_motor(self, msg):
        self.disable_motor = msg.data

    def brake_command_callback(self,msg):
        self.brake_command = msg.data

    def brake_status_callback(self,msg):
        self.brake_status = msg.data

    def update_ambient_temperature(self):
        while True:
            try:
                self.mc_lf_node.transmit_ambient_temperature(self.ambient_temperature_F)
                self.mc_lb_node.transmit_ambient_temperature(self.ambient_temperature_F)
                self.mc_rf_node.transmit_ambient_temperature(self.ambient_temperature_F)
                self.mc_rb_node.transmit_ambient_temperature(self.ambient_temperature_F)

                time.sleep(10)
            except Exception as error:
                rospy.logerr("Ignore this error when power-cycling motor controllers. The update_ambient_temperature function from a thread of motor_controller_network.py raised an error, which says %s", error)
                time.sleep(1)
                continue
            

    def set_rpm(self,msg):
        self.left_front_rpm = msg.left_front
        self.left_back_rpm = msg.left_back
        self.right_front_rpm = msg.right_front
        self.right_back_rpm = msg.right_back

    def enable_power_for_all_motors(self):
        self.mc_lf_node.enable_power_if_disabled()
        self.mc_lb_node.enable_power_if_disabled()
        self.mc_rf_node.enable_power_if_disabled()
        self.mc_rb_node.enable_power_if_disabled()

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

    def are_all_measured_wheel_rpm_below_this(self, rpm):
        return (
            abs(self.mc_lb_node.wheel_rpm_actual) < rpm and
            abs(self.mc_lf_node.wheel_rpm_actual) < rpm and
            abs(self.mc_rf_node.wheel_rpm_actual) < rpm and
            abs(self.mc_rb_node.wheel_rpm_actual) < rpm
        )

    def is_any_measured_wheel_rpm_above_this(self, rpm):
        return (
            abs(self.mc_lb_node.wheel_rpm_actual) > rpm or
            abs(self.mc_lf_node.wheel_rpm_actual) > rpm or
            abs(self.mc_rf_node.wheel_rpm_actual) > rpm or
            abs(self.mc_rb_node.wheel_rpm_actual) > rpm
        )

    def drive(self):
        while True:
            try:
                time.sleep(TIME_INTERVAL)
                if self.overseer_state == STOPPED:
                    self.enable_power_for_all_motors()
                    self.send_zero_rpm_to_all_motors()
                elif self.overseer_state == DESCENDING or self.overseer_state == RETURN_TO_START:
                    self.enable_power_for_all_motors()

                    self.mc_lf_node.spin(self.left_front_rpm)
                    self.mc_lb_node.spin(self.left_back_rpm)
                    self.mc_rf_node.spin(self.right_front_rpm)
                    self.mc_rb_node.spin(self.right_back_rpm)
                elif self.overseer_state == E_STOPPED:
                    self.enable_power_for_all_motors()
                    self.send_zero_rpm_to_all_motors()
                elif self.overseer_state == MANUAL:
                    if self.left_front_rpm == 0 and self.left_back_rpm == 0 and self.right_front_rpm == 0 and self.right_back_rpm == 0:
                        if self.does_brake_when_stopped:
                            self.enable_power_for_all_motors()
                            self.send_zero_rpm_to_all_motors()
                        else:
                            if self.is_any_measured_wheel_rpm_above_this(450):
                                self.enable_power_for_all_motors()
                                self.send_zero_rpm_to_all_motors()
                            else:
                                self.quick_stop_all_motors()
                    else:
                        self.enable_power_for_all_motors()

                        self.mc_lf_node.spin(self.left_front_rpm)
                        self.mc_lb_node.spin(self.left_back_rpm)
                        self.mc_rf_node.spin(self.right_front_rpm)
                        self.mc_rb_node.spin(self.right_back_rpm)
                elif self.overseer_state == AUTO:
                    if self.previous_state != self.overseer_state:
                        self.disable_motor = False
                    if self.disable_motor:
                        self.quick_stop_all_motors()
                    else:
                        if self.brake_command == True and self.brake_status != 2:
                            self.relax_motors_while_braking()
                        else:
                            self.enable_power_for_all_motors()
                            self.mc_lf_node.spin(self.left_front_rpm)
                            self.mc_lb_node.spin(self.left_back_rpm)
                            self.mc_rf_node.spin(self.right_front_rpm)
                            self.mc_rb_node.spin(self.right_back_rpm)
                elif self.overseer_state == IDLE:
                    if self.is_any_measured_wheel_rpm_above_this(450):
                        self.enable_power_for_all_motors()
                        self.send_zero_rpm_to_all_motors()
                    else:
                        self.quick_stop_all_motors()
                self.previous_state = self.overseer_state
            except Exception as error:
                rospy.logerr("Ignore this error when power-cycling motor controllers. The drive function from a thread of motor_controller_network.py raised an error, which says %s", error)
                time.sleep(1)
                continue

if __name__ ==  '__main__':
    node = rospy.init_node('can_interface')

    motor_controller_network = MotorControllerNetwork()
    
    rospy.spin()
        


    
