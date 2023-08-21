#!/bin/bash
#Script to use Jetson GPIO pins

source /opt/ros/melodic/setup.bash #To recognize ROS commands

#Tell Tegra’s GPIO driver that these pins are to be used as GPIO 
if [ ! -d /sys/class/gpio/gpio388 ]; then
	echo 388 > /sys/class/gpio/export #Pin 7
	echo 298 > /sys/class/gpio/export #Pin 8
	echo 480 > /sys/class/gpio/export #Pin 9
	echo 486 > /sys/class/gpio/export #Pin 10
fi

#Set Pin 8,9 as output pins (Pin 7 and 10 are already input pins by default).
echo  out > /sys/class/gpio/gpio298/direction
echo  out > /sys/class/gpio/gpio480/direction

#Set pins to read 0 when LOW, 1 otherwise
echo  0 > /sys/class/gpio/gpio388/active_low
echo  0 > /sys/class/gpio/gpio298/active_low
echo  0 > /sys/class/gpio/gpio480/active_low
echo  0 > /sys/class/gpio/gpio486/active_low

