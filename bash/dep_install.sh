#!/bin/bash
sudo apt install ros-melodic-tf
sudo apt install ros-melodic-gps-common
rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install build-essential libssl-dev libffi-dev python-dev
pip install --upgrade setuptools
pip install --upgrade pip
pip install canopen==1.1.0
pip install autobahn==19.11.2
pip install twisted==20.3.0
