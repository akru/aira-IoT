#!/bin/sh

mkdir $1
SCRIPT_NAME=`readlink -f $0`
AIRA_IOT_DIR=`dirname \`dirname ${SCRIPT_NAME}\``
INSTALL_DIR=`readlink -f $1`
echo "Install to ${INSTALL_DIR}..."

mkdir -p ${INSTALL_DIR}/src && cd ${INSTALL_DIR}/src
catkin_init_workspace
ln -s ${AIRA_IOT_DIR}
git clone https://github.com/mavlink/mavros.git
git clone https://github.com/PX4/mav_comm.git

ln -s ${AIRA_IOT_DIR}/patches/Findmavlink.cmake mavros/libmavconn/cmake/Modules/Findmavlink.cmake

cd ..

git clone https://github.com/mavlink/mavlink.git
mkdir mavlink_build && cd mavlink_build
cmake ../mavlink -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}/devel && make install
cd ..

#catkin_make
