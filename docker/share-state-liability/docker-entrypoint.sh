#!/bin/bash

export IPFS_IP=`getent hosts ${IPFS_NODE} | awk '{ print $1 }' | head -n1`

source aira-IoT/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch & 
rosrun robot_liability liability.py &

sleep 30
export NODE_PATH=$NODE_PATH:/usr/local/lib/node_modules
exec listen-market.js
