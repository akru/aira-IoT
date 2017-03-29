#!/bin/sh

echo Building contracts...
if [ ! -x .cache ]; then mkdir .cache; fi
aira-deploy -O -I./src/aira_ros_bridge/aira_ros_bridge -I./core --abi

echo Building ROS components...
cd src && catkin_init_workspace
cd ..  && catkin_make

echo All done!
