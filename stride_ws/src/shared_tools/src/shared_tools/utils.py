# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

import time
import csv

def find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed):
    dt = time.time() - initial_time
    sign = (1 if (speed_goal - initial_speed) >= 0 else -1)
    dy = min(abs(speed_goal-initial_speed), abs(speed_rate)*dt)
    return initial_speed + sign*dy

class WriteCSV(object):
    def __init__(self, name):
        self.fn = name

    def open(self):
        self.file = open(self.fn, 'w')
        self.csv_writer = csv.writer(self.file, delimiter=',')
        
    def write(self, row):
        self.csv_writer.writerow(row)

    def close(self):
        self.file.close()