#!/bin/bash
#==============================================================================
# Title: install.sh
# Description: Install the package in order to be able to use it from other 
# catkin packages.
#==============================================================================

export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/
mkdir build && cd build
cmake ..
make -j4
sudo make install -j4
cd .. 
