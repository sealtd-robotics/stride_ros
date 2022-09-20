#!/bin/bash
dir_init=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt-get update
sudo apt-get install python-pip
sudo apt-get install build-essential libssl-dev libffi-dev python-dev
# sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-melodic-tf
sudo apt install ros-melodic-tf2-geometry-msgs

python -m pip install --upgrade setuptools
python -m pip install --upgrade pip
cd $dir_init
python -m pip install -r ./requirements.txt
