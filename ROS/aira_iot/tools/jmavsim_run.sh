#!/bin/sh

git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule init
git submodule update
make posix -j8
make posix_sitl_default jmavsim
