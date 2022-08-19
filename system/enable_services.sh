#!/bin/bash

CURRENT_USER=$SUDO_USER
ROS_DISTRO=melodic

# Copy .service file to /etc/systemd/system
# cp roscore.service /etc/systemd/system/$ROS_DISTRO@roscore.service
cp can_bus.service /etc/systemd/system/$CURRENT_USER@can_bus.service

# Enable service to start at boot
# systemctl enable $ROS_DISTRO@roscore.service
systemctl enable $CURRENT_USER@can_bus.service
