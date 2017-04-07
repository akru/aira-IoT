#!/bin/bash

sed -i "0,/localhost/{s/localhost/${PARITY_NODE}/}" aira-IoT/src/aira_ros_bridge/aira_ros_bridge/lib/aira_bridge.js
export IPFS_IP=`getent hosts ${IPFS_NODE} | awk '{ print $1 }' | head -n1`

if [ -z ${LIABILITY_CONTRACT} ]; then
    echo "ERROR: No liability contract specified! Terminate..."
    exit 1
else
    echo "Liability: ${LIABILITY_CONTRACT}"
fi

source aira-IoT/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch & 

sleep 5
roslaunch aira_ros_bridge node.launch contract_address:=${LIABILITY_CONTRACT} &

sleep 5
rosrun robot_liability liability.py &

while [ 1 ]; do
    sleep $PERIOD
    ipfs --api=/ip4/${IPFS_IP}/tcp/5001 add -rq state | tail -n1 | xargs rostopic pub --once result_ipfs std_msgs/String
done
