#!/bin/bash
#==============================================================================
# Title: install.sh
# Description: Install everything necessary for the project, based on the install
# script written by Daniyal Shahrokhian <daniyal@kth.se> on 2017.04.28
#==============================================================================

# Exit script if any command fails
set -e 
set -o pipefail

if [ $# -ne 0 ]
  then
    echo "Usage: install.sh"
    exit 1
fi

# Creating a Library folder on the desktop
cd ~/Desktop/Libraries

# ROS Installation
echo "Installing ROS..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "Done installing ROS..."

# Essential Dependencies
echo "Installing Essential dependencies..."
sudo apt-get -y update
sudo apt-get -y install build-essential
sudo apt-get -y install cmake
sudo apt-get -y install libopenblas-dev liblapack-dev
sudo apt-get -y install git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get -y install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
echo "Essential dependencies installed."

# OpenCV Dependency
echo "Downloading OpenCV..."
wget https://github.com/opencv/opencv/archive/3.4.0.zip
unzip 3.4.0.zip
cd opencv-3.4.0
mkdir -p build
cd build
echo "Installing OpenCV..."
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_CUDA=OFF -D BUILD_SHARED_LIBS=OFF ..
make -j4
sudo make install
cd ../..
rm 3.4.0.zip
#sudo rm -r opencv-3.4.0
echo "OpenCV installed."

# dlib dependecy
echo "Downloading dlib"
wget http://dlib.net/files/dlib-19.15.tar.bz2;
tar xf dlib-19.15.tar.bz2;
cd dlib-19.15;
mkdir -p build;
cd build;
echo "Installing dlib"
cmake ..;
cmake --build . --config Release;
sudo make install;
sudo ldconfig;
cd ../..;    
#rm -r dlib-19.15.tar.bz2
echo "dlib installed"

# Boost C++ Dependency
echo "Installing Boost..."
sudo apt-get install libboost-all-dev
echo "Boost installed."

# OpenFace installation
echo "Installing OpenFace and models..."
git clone https://github.com/TadasBaltrusaitis/OpenFace.git
cd OpenFace
sudo ./download_models.sh
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make
sudo make install
cd ../..
echo "OpenFace successfully installed."
