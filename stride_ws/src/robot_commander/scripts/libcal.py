#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from math import cos, sin, sqrt, pi, radians
import numpy as np

_e = 0.0818191908426
_R = 6378137
 
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