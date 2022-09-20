#!/bin/bash

CURRENT_USER=$SUDO_USER

dir_init=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $dir_init

# Copy .service file to /etc/systemd/system
cp web_server.service /etc/systemd/system/$CURRENT_USER@web_server.service

# Enable service to start at boot
systemctl enable $CURRENT_USER@web_server.service