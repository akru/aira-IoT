#!/bin/bash

export IPFS_IP=`getent hosts ${IPFS_NODE} | awk '{ print $1 }' | head -n1`

if [ -z $1 ]; then
    echo "ERROR: No liability contract specified! Terminate..."
    exit 1
else
    echo "Liability: $1"
fi

source aira-IoT/devel/setup.bash

roslaunch aira_ros_bridge node.launch contract_address:=$1 &
sleep 5

ipfs --api=/ip4/${IPFS_IP}/tcp/5001 add -rq state | tail -n1 | xargs rostopic pub --once result_ipfs std_msgs/String

kill $!
