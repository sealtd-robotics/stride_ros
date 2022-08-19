#!/bin/bash

# scp -r stride_ws/src/* nvidia@195.0.0.10:~/stride_ros/stride_ws/src/

dir_init=$HOME
echo "  -> Initial dir = $dir_init"

cd "$dir_work"
unzip stride_ros-main.zip && mv stride_ros-main stride_ros