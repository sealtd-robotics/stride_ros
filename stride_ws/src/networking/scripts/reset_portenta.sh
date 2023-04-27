#!/bin/bash
#script to change output GPIO pin (Pin 9) to HIGH and then LOW which resets the Portenta

echo 1 > /sys/class/gpio/gpio480/value
sleep 0.1
echo 0 > /sys/class/gpio/gpio480/value