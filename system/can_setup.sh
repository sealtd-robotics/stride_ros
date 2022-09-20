#!/bin/bash
# This is the bash-script to set up system settings
# Setting up can0 to 250K baudrate

modprobe mttcan

if ifconfig -a | grep -q can0; then
    modprobe can
    modprobe can_raw
    ip link set can0 type can bitrate 1000000 dbitrate 2000000 berr-reporting on fd on
    ip link set up can0
    echo "Found CAN device. Setup 1000K at boot"
else
    echo "No CAN device found"
fi
