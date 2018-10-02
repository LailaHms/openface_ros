#!/bin/bash
#==============================================================================
# Title: install.sh
# Description: Install the package in order to be able to use it from other 
# catkin packages.
#==============================================================================

# Exit script if any command fails
set -e 
set -o pipefail

if [ $# -ne 0 ]
  then
    echo "Usage: install.sh"
    exit 1
fi


export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/
mkdir build
cd build
cmake ..
make -j4
sudo make install -j4
cd .. 
