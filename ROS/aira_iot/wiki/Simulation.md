# Install simulation enviroment

## Install ROS

Install `ros-indigo-desktop-full` for your distro.
Install `ros-indigo-control_toolbox` for your distro.

## Install SITL Enviroment

### MAVLink&PX4 ROS node install

Run `aira_iot/tools/mavlink_build_ws.sh mav_ws`,
this script build MAVLink catkin workspace for the ROS.

### ArduCopter SITL

    $ git clone https://github.com/akru/ardupilot.git

Run:

    $ export PATH=$PATH:~/ardupilot/Tools/autotest
    $ fg_quad_view.sh & sim_vehicle.sh --map --console

This instructions create basic sim enviroment with MAVLink
control interface. mavros package is able to use this.
