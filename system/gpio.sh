#!/bin/bash
#script to change output GPIO pins (Pin 8, 9) based on input GPIO pin (Pin 7)

source /opt/ros/melodic/setup.bash #To recognize ROS commands

#Tell Tegra’s GPIO driver that these pins are to be used as GPIO 
if [ ! -d /sys/class/gpio/gpio388 ]; then
	echo 388 > /sys/class/gpio/export #Pin 7
	echo 298 > /sys/class/gpio/export #Pin 8
	echo 480 > /sys/class/gpio/export #Pin 9
fi

#Set Pin 8,9 as output (Pin 7 is already input cause thats the default setting)
echo  out > /sys/class/gpio/gpio298/direction
echo  out > /sys/class/gpio/gpio480/direction

#Set pins to read 0 when LOW, 1 otherwise
echo  0 > /sys/class/gpio/gpio388/active_low
echo  0 > /sys/class/gpio/gpio298/active_low
echo  0 > /sys/class/gpio/gpio480/active_low

while [ True ]; do
	msg=$(rostopic echo -n1 /portenta_heartbeat | cut -c 7-)
	if [ $msg == False ]; then #if Portenta is dead
			echo 1 > /sys/class/gpio/gpio298/value #set Pin 8 HIGH                 
	elif [ $msg == True ]; then #if Portenta is alive
			echo 0 > /sys/class/gpio/gpio298/value #set outputs LOW
			echo 0 > /sys/class/gpio/gpio480/value
	fi
	sleep 0.1 
done 


