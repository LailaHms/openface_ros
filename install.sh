#!/bin/bash
#==============================================================================
# Title: install.sh
# Description: Install the package in order to be able to use it from other 
# catkin packages.
#==============================================================================


RED='\033[0;31m'
NC='\033[0m' # No Color


#Remove the build folder if it already exists
sudo rm -R build

# Export the python path, necessary to avoid issues with bad python package linking

export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/

# Exit script if any command fails
set -e 
set -o pipefail

if [ $# -ne 0 ]
  then
    echo "Usage: install.sh"
    exit 1
fi


echo "-------------------------------------------------"
echo -e "${RED}\n1. Install Dependencies${NC}\n"
echo "-------------------------------------------------"


chmod +x ./install_dependencies.sh
sudo ./install_dependencies.sh


echo "-------------------------------------------------"
echo -e "${RED}\n1. Install The OpenFace Ros Wrappers${NC}\n"
echo "-------------------------------------------------"

mkdir build
cd build
cmake ..
make -j4
sudo make install -j4
cd .. 
