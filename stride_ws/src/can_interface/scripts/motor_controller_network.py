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
        self.winding_temperature = 0
        self.state = 0
        self.wheel_rpm_actual = 0

        self.node = node
        self.topic_name = topic_name

        self.heartbeat_arrival_time = self.get_time_now_in_ms() + 5000 # added some time buffer for startup
        self.overseer_state = 0
        self.sdo_ambient_temperature = self.node.sdo[0x232A][0x08]

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

        # Subscribers
        # rospy.Subscriber('/overseer/state', Int32, self.overseer_state_callback)

        # Heartbeat timeout thread
        self.heartbeat_thread = threading.Thread(target=self.monitor_heartbeat)
        self.heartbeat_thread.setDaemon(True)
        self.heartbeat_thread.start()

    def transmit_ambient_temperature(self, degree_F):
        degree_C = (degree_F - 32) * 5 / 9
        self.sdo_ambient_temperature.raw = degree_C

        time.sleep(0.05) 

    def monitor_heartbeat(self):
        while True:
            if self.get_time_now_in_ms() - self.heartbeat_arrival_time > 1000:
                self.heartbeat_timeout_publisher.publish(True)
            else:
                self.heartbeat_timeout_publisher.publish(False)
            time.sleep(0.1)

    # May not be needed anymore....... Test it !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # def overseer_state_callback(self, new_state):
    #     if self.overseer_state == new_state.data:
    #         return

    #     is_current_state_inoperable = self.overseer_state == 3 or \
    #                                     self.overseer_state == 4 or \
    #                                     self.overseer_state == 5

    #     # if overseer state changes from inoperable to manual or auto, run recovery steps
    #     if is_current_state_inoperable and (new_state.data == 1 or new_state.data == 2):
    #         self.fault_reset_command()
    #         self.change_nmt_state('OPERATIONAL')

    #     self.overseer_state = new_state.data

    def change_nmt_state(self, nmt_state):
        # nmt_state is a string
        self.node.nmt.state = nmt_state
        time.sleep(0.02)

    # def fault_reset_command(self):
    #     self.node.rpdo[2][0].raw = int('10000000',2) # 1000 0000
    #     self.node.rpdo[2].transmit()
    #     time.sleep(0.02)

    def disable_enable_power(self):
        # disable and then re-enable power
        # can also be used for simply enabling power

        # Send a "shut down" command to go to "Ready to Switch On" state, from either "Operation Enabled" or "Switch on Disabled"
        self.node.rpdo[2][0].raw = int('0110',2)
        self.node.rpdo[2].transmit()
        time.sleep(0.02)

        # Then send a "Enable Operation" command to go to the "Operation Enable" state
        # This state enables power to motor
        self.node.rpdo[2][0].raw = int('1111',2)
        self.node.rpdo[2].transmit()
        time.sleep(0.02)

    def enable_power_if_disabled(self):
        # return when there is power already
        if self.state == 39 or self.state == 55: # Operation Enabled can be 39 or 55 
            return
        
        # The commands inside this function are same for enabling power
        self.disable_enable_power()

    def quick_stop_controlword(self):
        self.node.rpdo[2][0].raw = int('10',2)
        self.node.rpdo[2].transmit()
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

        # 1
        winding_temperature = tpdo2[1].raw
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
        rospy.Subscriber('/robot_temperature', Int32, self.set_ambient_temperature, queue_size=1)

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
        return (self.overseer_state == 1 or self.overseer_state == 3 or self.overseer_state == 5) and \
                self.are_all_measured_wheel_rpm_below_this(10)

    def relax_motors(self):
        interval = 3
        while True:
            time.sleep(interval)
            if self.can_relax():
                self.mc_lf_node.disable_enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_lb_node.disable_enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_rf_node.disable_enable_power()

            time.sleep(interval)
            if self.can_relax():
                self.mc_rb_node.disable_enable_power()

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

    def update_ambient_temperature(self):
        while True:
            self.mc_lf_node.transmit_ambient_temperature(self.ambient_temperature_F)
            self.mc_lb_node.transmit_ambient_temperature(self.ambient_temperature_F)
            self.mc_rf_node.transmit_ambient_temperature(self.ambient_temperature_F)
            self.mc_rb_node.transmit_ambient_temperature(self.ambient_temperature_F)

            time.sleep(10)
            

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
            time.sleep(0.02)
            if self.overseer_state == 5: # 5: STOPPED
                self.enable_power_for_all_motors()
                self.send_zero_rpm_to_all_motors()
            elif self.overseer_state == 6: # 6: DECENDING
                self.enable_power_for_all_motors()

                self.mc_lf_node.spin(self.left_front_rpm)
                self.mc_lb_node.spin(self.left_back_rpm)
                self.mc_rf_node.spin(self.right_front_rpm)
                self.mc_rb_node.spin(self.right_back_rpm)
            elif self.overseer_state == 3: # 3: ESTOPPED
                # self.quick_stop_all_motors()
                self.enable_power_for_all_motors()
                self.send_zero_rpm_to_all_motors()
            elif self.overseer_state == 1: # MANUAL state
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
            elif self.overseer_state == 2: # AUTO state
                self.enable_power_for_all_motors()

                self.mc_lf_node.spin(self.left_front_rpm)
                self.mc_lb_node.spin(self.left_back_rpm)
                self.mc_rf_node.spin(self.right_front_rpm)
                self.mc_rb_node.spin(self.right_back_rpm)
            elif self.overseer_state == 7: # IDLE state
                if self.is_any_measured_wheel_rpm_above_this(450):
                    self.enable_power_for_all_motors()
                    self.send_zero_rpm_to_all_motors()
                else:
                    self.quick_stop_all_motors()

if __name__ ==  '__main__':
    node = rospy.init_node('can_interface')

    motor_controller_network = MotorControllerNetwork()
    
    rospy.spin()
        


    

