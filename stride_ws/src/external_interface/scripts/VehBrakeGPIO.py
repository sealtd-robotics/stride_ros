#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

gpio_path = "/sys/class/gpio/gpio486/value"
# veh_brake_pub = rospy.Publisher("/vehicle_brake", Bool, queue_size=10)

# if __name__ == '__main__':
#     rospy.init_node('VehBrakeGPIO', anonymous=True)
#     rate = rospy.Rate(50) # 50hz    
#     while not rospy.is_shutdown():
#         with open(gpio_path, "r") as gpio_file:
#             pin_value = gpio_file.read().strip()
#             if pin_value == "0":
#                 veh_brake_pub.publish(False)
#             elif pin_value == "1":
#                 veh_brake_pub.publish(True)
#         rate.sleep()
