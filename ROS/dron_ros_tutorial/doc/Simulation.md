# Install simulation enviroment

## Install ROS

Install `ros-indigo-desktop-full` for your distro.

## Install SITL enviroment

### MAVLink & Ardupilot ROS node install

Run `dron_ros_tutorial/tools/mavlink_build_ws.sh mav_ws`,
this script build MAVLink catkin workspace for the ROS.

### ArduCopter SITL

Clone the Ardupilot repository:

    $ git clone https://github.com/akru/ardupilot.git
    $ cd ardupilot

Patch for RC-less using:

    $ git checkout 36b405fb0b2b3ef77952e8c9f170dbcf1976cb30
    $ patch -p1 < /path/to/dron_ros_tutorial/patches/ardupilot_rcless.patch

Run the SITL:

    $ cd ArduCopter
    $ export PATH=$PATH:../ardupilot/Tools/autotest
    $ fg_quad_view.sh & sim_vehicle.sh --map --console

This instructions create basic simulation enviroment with MAVLink
control interface. mavros package is able to use this.
