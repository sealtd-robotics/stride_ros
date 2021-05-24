#!/bin/bash
rosnode kill rosbridge_websocket
sleep 0.2

rosnode kill rosapi
sleep 0.2

fuser -k -n tcp 9090
sleep 0.2

roslaunch rosbridge_server rosbridge_websocket.launch