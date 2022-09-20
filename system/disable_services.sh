#!/bin/bash

CURRENT_USER=$SUDO_USER
ROS_DISTRO=melodic

# Disable services
systemctl disable $ROS_DISTRO@roscore.service
systemctl disable $CURRENT_USER@can_bus.service
systemctl disable $CURRENT_USER@stride_top_level.service

# Remove service files from /etc/systemd/system
rm /etc/systemd/system/$ROS_DISTRO@roscore.service
rm /etc/systemd/system/$CURRENT_USER@can_bus.service
rm /etc/systemd/system/$CURRENT_USER@stride_top_level.service