#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from math import cos, sin, sqrt, pi, radians
import numpy as np
from utils import WriteCSV

_e = 0.0818191908426
_R = 6378137

def kph2mps(kph):
    return kph*10.0/36.0


def mps2kph(mps):
    return mps*3.6


class LL_NE(object):
    def __init__(self, refLat=0, refLong=0):
        self.update(refLat, refLong)

    def update(self, refLat, refLong):
        self.RefLat = refLat
        self.RefLong = refLong
        self.EN_factors(refLat)

    def EN_factors(self, RefLat):
        self.eFactor = cos(RefLat*pi/180)*_R/sqrt(1-(sin(RefLat*pi/180)**2*_e**2))*pi/180
        self.nFactor = (1-_e**2)*_R/((1-(sin(RefLat*pi/180)**2*_e**2))*sqrt(1-(sin(RefLat*pi/180)**2*_e**2)))*pi/180

    def LL2NE(self, latitude, longitude):
        pos_east = (longitude - self.RefLong) * self.eFactor
        pos_north = (latitude - self.RefLat) * self.nFactor
        return pos_north, pos_east

    def NE2LL(self, pos_north, pos_east):
        longitude = (pos_east/self.eFactor) + self.RefLong 
        latitude = (pos_north/self.nFactor) + self.RefLat
        return latitude, longitude


class CheckBoundariesEnter(object):
    """"
    Algorithm idea, check if the vehicle enters the triangle at the trigger location.
    Inspired by this concept: https://blackpawn.com/texts/pointinpoly/
    """
    def __init__(self, trigger_heading):
        self.trigger_heading = trigger_heading
        self.generate_boundaries(trigger_heading)

    def generate_boundaries(self, trigger_heading, dist=2.0):
        """
        Generate triangle vertices from origin (0,0) with assigned heading

        Parameters:
            trigger_heading - heading in degrees of trigger location
            dist - 
        Return:
            p1,p2,p3 - coordinates (x,y) of triangle vertices in numpy array format
                    origin reaches p3 at trigger_heading
                    origin reaches p1 at trigger_heading - 90
                    origin reaches p2 at trigger_heading + 90
        """
        fix_heading_rad = radians(trigger_heading)
        self.p3 = np.array([dist*sin(fix_heading_rad), dist*cos(fix_heading_rad)])
        t1 = fix_heading_rad-pi/2
        self.p1 = np.array([dist*sin(t1), dist*cos(t1)])
        t2 = fix_heading_rad+pi/2
        self.p2 = np.array([dist*sin(t2), dist*cos(t2)])

    def in_boundaries(self, p):
        """
        Check if point p is within boundaries 
        """
        if self.same_side(p, self.p3, self.p1, self.p2) and \
            self.same_side(p, self.p1, self.p2, self.p3) and \
            self.same_side(p, self.p2, self.p1, self.p3):
            return True
        return False

    @staticmethod 
    def same_side(p1, p2, a, b):
        """ Check if p1 and p2 are on the same side of line ab"""
        cp1 = np.cross(b-a, p1-a)
        cp2 = np.cross(b-a, p2-a)
        if np.dot(cp1, cp2) >= 0:
            return True
        return False


class Compensation_Errors(object):
    PAST_TRIGGER_POINT = -1
    PRE_CAL_PROBLEM = -2
    INVALID = -3
    def __init__(self):
        pass


class Compensation(object):
    def __init__(self, path_to_follow):
        self.latlong_path = path_to_follow
        self.collision_east_proj = -1
        self.collision_north_proj = -1

        _lat = path_to_follow['latitudes']
        _long = path_to_follow['longitudes']
        self.llne = LL_NE(_lat[0], _long[0])
        self.north, self.east = [], []
        for i in range(len(_lat)):
            north, east = self.llne.LL2NE(_lat[i], _long[i])
            self.north.append(north)
            self.east.append(east)

        # self.collision_north, self.collision_east = self.llne.LL2NE(collision_lat, collision_long)

    def pre_collision_calc(self, collision_lat, collision_long):
        col_north, col_east = self.llne.LL2NE(collision_lat, collision_long)
        self.pre_collision_index = len(self.north) - 1
        self.dist_intervals = []

        for i in range(len(self.north) - 1):
            y = self.north[i+1] - self.north[i]
            x = self.east[i+1] - self.east[i]
            d = np.sqrt(y*y + x*x)

            self.dist_intervals.append(d)

            # slope of current index to collision point
            if x == 0.0:
                x = 0.0001
            m = y/x
            m_perp = -1/m
            k_cur = y - m_perp*x
            k_col = (self.north[i+1] - col_north) - m_perp*(self.east[i+1] - col_east)

            # Check if col point and current index not in the same side 
            if k_cur*k_col > 0:
                self.pre_collision_index = i
                break

        # print("Pre collision index: ", self.pre_collision_index)
        # print("Intervals: ", self.dist_intervals)
        # find the col dist from closest index
        if self.pre_collision_index < (len(self.north) - 1):
            b = self.dist_intervals[self.pre_collision_index]
            ay = col_north - self.north[self.pre_collision_index]
            ax = col_east - self.east[self.pre_collision_index]
            by = self.north[self.pre_collision_index+1] - self.north[self.pre_collision_index]
            bx = self.east[self.pre_collision_index+1] - self.east[self.pre_collision_index]
            col_dist_from_closet_index = (ax*bx + ay*by)/b

            cx = col_dist_from_closet_index * bx / b
            cy = col_dist_from_closet_index * by / b
            self.collision_east_proj = cx + self.east[self.pre_collision_index]
            self.collision_north_proj = cy + self.north[self.pre_collision_index]
        else: # the col point should always be within path
            return False

        # print("Col dist from closet index: ", col_dist_from_closet_index)

        # assign collision distance
        # w = WriteCSV('/home/nvidia2/stride_ros/stride_ws/test_log/comp_dist.csv')
        # w.open()

        self.dtc_array = [0]*len(self.north)
        self.dtc_array[0] = sum(self.dist_intervals[0:self.pre_collision_index]) \
                                                    + col_dist_from_closet_index 
        # w.write([0,self.dtc_array[0]])
        for i in range(1,self.pre_collision_index+1):
            self.dtc_array[i] = self.dtc_array[i-1] - self.dist_intervals[i-1]
            # w.write([i, self.dtc_array[i]])
        # w.close()
        return True
    
    def find_current_index(self, lat, long, cur_index):
        """
        Find the starting index of the segment that the vehicle is in
        """
        _north, _east = self.llne.LL2NE(lat, long)

        if cur_index >= len(self.north) - 1:
            return cur_index

        for i in range(cur_index, len(self.north) - 1):
            y = self.north[i+1] - self.north[i]
            x = self.east[i+1] - self.east[i]
            d = np.sqrt(y*y + x*x)

            # slope of current index to collision point
            if x == 0.0:
                x = 0.0001
            m = y/x
            m_perp = -1/m
            k_cur = y - m_perp*x
            k_col = (self.north[i+1] - _north) - m_perp*(self.east[i+1] - _east)

            # Check if col point and current index not in the same side 
            if k_cur*k_col > 0:
                return i
        return len(self.north) - 1
            

    def dist_to_collision(self, veh_lat, veh_long, current_index):
        # check if pre_collision_calc did its work and valid
        # abort if default value doesn't change
        if self.collision_east_proj == -1:
            return Compensation_Errors().PRE_CAL_PROBLEM
        
        veh_north, veh_east = self.llne.LL2NE(veh_lat, veh_long)

        if current_index == self.pre_collision_index:
            # print("At pre-collision index check")
            # check if the vehicle pass the collision point
            # when the veh enters the same segnment path as collision point
            y = (self.collision_north_proj - self.north[current_index + 1])
            x = (self.collision_east_proj - self.east[current_index + 1])

            if x == 0.0:
                x = 0.0001
            m = y/x
            m_perp = -1/m
            k_next = y - m_perp*x
            k_veh = (self.collision_north_proj - veh_north) - m_perp*(self.collision_east_proj - veh_east)

            # Check if vehicle passed target, return -1 if true
            if k_next*k_veh > 0:
                return Compensation_Errors().PAST_TRIGGER_POINT
        elif current_index > self.pre_collision_index:
            # already passed the collision segment
            return Compensation_Errors().PAST_TRIGGER_POINT
        
        b = self.dist_intervals[current_index]
        ay = self.north[current_index + 1] - veh_north
        ax = self.east[current_index + 1] - veh_east
        by = self.north[current_index+1] - self.north[current_index]
        bx = self.east[current_index+1] - self.east[current_index]
        veh_dist_to_next_index = (ax*bx + ay*by)/b

        if current_index == self.pre_collision_index:
            dtc = self.dtc_array[current_index] - self.dist_intervals[current_index] + veh_dist_to_next_index
            # print("return pre-col index value")
            return dtc
        
        return self.dtc_array[current_index + 1] + veh_dist_to_next_index
    
    def time_to_collision(self, veh_lat, veh_long, veh_speed, current_index):
        dtc = self.dist_to_collision(veh_lat, veh_long, current_index)
        if dtc > 0:
            return dtc / veh_speed
        else:
            return dtc # this will return error code
    
    
