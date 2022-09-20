#!/bin/bash

CURRENT_USER=$SUDO_USER

# Disable services
systemctl disable $CURRENT_USER@web_server.service

# Remove service files from /etc/systemd/system
rm /etc/systemd/system/$CURRENT_USER@web_server.service