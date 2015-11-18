#!/bin/sh

mkdir $1
AIRA_IOT_DIR=`dirname \`readlink -f $0\``
INSTALL_DIR=`readlink -f $1`
echo "Install to ${INSTALL_DIR}..."

mkdir -p ${INSTALL_DIR}/src && cd ${INSTALL_DIR}/src
catkin_init_workspace
git clone https://github.com/mavlink/mavros.git
git clone https://github.com/PX4/mav_comm.git
#git clone https://github.com/PX4/Firmware.git
#git clone https://github.com/PX4/rotors_simulator.git
#git clone https://github.com/ethz-asl/glog_catkin.git
#git clone https://github.com/catkin/catkin_simple.git

#cd Firmware
#git submodule init
#git submodule update
#patch -p1 < ${AIRA_IOT_DIR}/patches/Firmware_catkin_make_fix.patch
#cd ..

#hg clone https://bitbucket.org/osrf/osrf-common
#hg clone https://bitbucket.org/osrf/sandia-hand
#hg clone https://bitbucket.org/osrf/drcsim

cd ..

git clone https://github.com/mavlink/mavlink.git
mkdir mavlink_build && cd mavlink_build
cmake ../mavlink -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}/devel && make install
cd ..

catkin_make
