#!/bin/bash

sed -i "0,/localhost/{s/localhost/${PARITY_NODE}/}" aira-IoT/src/aira_ros_bridge/aira_ros_bridge/lib/aira_bridge.js
export IPFS_IP=`getent hosts ${IPFS_NODE} | awk '{ print $1 }' | head -n1`

if [ -z ${MARKET_CONTRACT} ]; then
    echo "ERROR: No market contract specified! Terminate..."
    exit 1
else
    echo "Liability Market: ${MARKET_CONTRACT}"
fi

source aira-IoT/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch & 
rosrun robot_liability liability.py &

sleep 20
export NODE_PATH=$NODE_PATH:/usr/local/lib/node_modules
exec listen-market.js ${PARITY_NODE} ${MARKET_CONTRACT} 
