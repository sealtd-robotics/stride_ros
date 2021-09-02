# Abbreviations
# v: linear velocity
# w: angular velocity

import rospy
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from math import cos, sin, sqrt, pi

class PathFollower:
    def __init__(self):
        self.point_interval = 0.2   # meters
        self.look_ahead_points = 3

        self.current_path_index = 0
        self.overseer_state = 5      # 5: STOPPED state
        self.E_factor = 0
        self.N_factor = 0
        self.latitude_measured = 0
        self.longitude_measured = 0
        self.heading_measured = 0
        self.easts = np.array([])
        self.norths = np.array([])
        

        # Subscribers
        rospy.Subscriber('/overseer/state', Int32, self.subscriber_callback_1, queue_size=10)

        # GPS Subscribers
        rospy.Subscriber('/an_device/NavSatFix', NavSatFix, self.gps_subscriber_callback_1, queue_size=1)
        rospy.Subscriber('/an_device/heading', Float32, self.gps_subscriber_callback_2, queue_size=1)
        

        # rospy.Subscriber('/an_device/Twist', Twist, self.gps_subscriber_callback_3, queue_size=1)
    
    def EN_factors(self, lat_ref):
        e = 0.0818191908426
        R = 6378137

        E_factor = cos(lat_ref*pi/180)*R/sqrt(1-(sin(lat_ref*pi/180)**2*e**2))*pi/180

        N_factor = (1-e**2)*R/((1-(sin(lat_ref*pi/180)**2*e**2))*sqrt(1-(sin(lat_ref*pi/180)**2*e**2)))*pi/180
        
        return (E_factor, N_factor)
    
    def LL2NE(self, latitude, longitude, lat_ref, long_ref, E_factor, N_factor):
        pos_east = (longitude - long_ref) * E_factor
        pos_north = (latitude - lat_ref) * N_factor
        return (pos_north, pos_east)

    def load_path(self):
        file = '../../../path/' + 'stride_path_fan.txt'
        self.easts = []
        self.norths = []

        line_number = 1
        with open(file, 'r') as f:
            for line in f.readlines():
                # omit the first line in file
                if line_number == 1:
                    line_number += 1
                    continue
                
            lat_long = line.split()
            latitude = lat_long[0]
            longitude = lat_long[1]

            # make the first point a reference point
            if line_number == 1:
                lat_ref = latitude
                long_ref = longitude
                (self.E_factor, self.N_factor) = self.EN_factors(lat_ref)

            (pos_north, pos_east) = self.LL2NE(latitude, longitude, lat_ref, long_ref, self.E_factor, self.N_factor)

            self.easts.append(pos_east)
            self.norths.append(pos_north)

        self.is_path_loaded = True

        for i in range(0, len(self.easts)):
            print(self.easts[i], self.norths[i])

    def update_current_path_index(self):
        # Find slope of line connecting current index and the next index
        x1 = self.easts[self.current_path_index]
        y1 = self.norths[self.current_path_index]

        x2 = self.easts[self.current_path_index + 1]
        y2 = self.norths[self.current_path_index + 1]

        m = (y2 - y1) / (x2 - x1)

        # Slope of the perpendicular line to the original line
        m_perp = -1/m

        # point-slope form for current index point
        k1 = m_perp * (x1 - x2) / (y1 - y2)

        # point-slope form for current location of robot
        k2 = m_perp * ()

        # if (x1, y1) and the current location of robot are on the opposite sides of the perpendicular line,
        # increment self.current_path_index


    # Subscriber callbacks
    def subscriber_callback_1(self, msg):
        self.overseer_state = msg.data

    # GPS subscriber callbacks
    def gps_subscriber_callback_1(self, msg):
        self.latitude_measured = msg.latitude     # degree
        self.longitude_measured = msg.longitude   # degree

    def gps_subscriber_callback_2(self, msg):
        self.heading_measured = msg.data    # radian

    # def gps_subscriber_callback_3(self, msg):
    #     self.linear_speed_measured = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
    #     self.yaw_velocity_measured = msg.angular.z

    def find_velocities(self):
        v = 1
        w = 2
        return v, w

if __name__ ==  '__main__':
    node = rospy.init_node('path_follower')

    path_follower = PathFollower()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if path_follower.overseer_state == 2:    # 2 is the autonomous (AUTO) state
            if 
        rate.sleep() 
