#!/bin/bash
#==============================================================================
# Title: install.sh
# Description: Install everything necessary for the project, based on the install
# script written by Daniyal Shahrokhian <daniyal@kth.se> on 2017.04.28
#==============================================================================

install_ros = false
CYAN='\033[0;36m'
NC='\033[0m' # No Color


# Exit script if any command fails
set -e
set -o pipefail

if [ $# -ne 0 ]
  then
    echo "Usage: install.sh"
    exit 1
fi

# Creating a Library folder on the desktop
if [ ! -d ~/Desktop/Libraries ]; then
  mkdir -p ~/Desktop/Libraries;
fi

cd ~/Desktop/Libraries

# ROS Installation
if [ "$install_ros" = true ] ; then
	echo -e "${CYAN}Installing ROS...${NC}"
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
	echo -e "${CYAN}Done installing ROS...${NC}"
else
	echo -e "${CYAN}Not Installing ROS...${NC}"

fi

# Essential Dependencies
echo -e "${CYAN}Installing Essential dependencies...${NC}"
sudo apt-get -y update
sudo apt-get -y install build-essential
sudo apt-get -y install cmake
sudo apt-get -y install libopenblas-dev liblapack-dev
sudo apt-get -y install git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get -y install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev python-scipy
echo -e "${CYAN}Essential dependencies installed.${NC}"

# OpenCV Dependency
echo -e "${CYAN}Downloading OpenCV...${NC}"
wget https://github.com/opencv/opencv/archive/3.4.0.zip
unzip 3.4.0.zip
cd opencv-3.4.0
mkdir -p build
cd build
echo -e "${CYAN}Installing OpenCV...${NC}"
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_CUDA=OFF -D BUILD_SHARED_LIBS=OFF ..
make -j4
sudo make install
cd ../..
rm 3.4.0.zip
#sudo rm -r opencv-3.4.0
echo -e "${CYAN}OpenCV installed.${NC}"

# dlib dependecy
echo -e "${CYAN}Downloading dlib${NC}"
wget http://dlib.net/files/dlib-19.15.tar.bz2;
tar xf dlib-19.15.tar.bz2;
cd dlib-19.15;
mkdir -p build;
cd build;
echo -e "${CYAN}Installing dlib${NC}"
cmake ..;
cmake --build . --config Release;
sudo make install;
sudo ldconfig;
cd ../..;
#rm -r dlib-19.15.tar.bz2
echo -e "${CYAN}dlib installed${NC}"

# Boost C++ Dependency
echo -e "${CYAN}Installing Boost...${NC}"
sudo apt-get install libboost-all-dev
echo -e "${CYAN}Boost installed.${NC}"

# OpenFace installation
echo -e "${CYAN}Installing OpenFace and models...${NC}"
git clone https://github.com/TadasBaltrusaitis/OpenFace.git
cd OpenFace
sudo chmod +x ./download_models.sh
sudo ./download_models.sh
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make
sudo make install
cd ../..
echo -e "${CYAN}OpenFace successfully installed.${NC}"

# install usb cam node and audio common for audio and video recordings
cd ~/catkin_ws/src
sudo apt-get install gstreamer1.0-libav
sudo apt-get install libgstreamer1.0-0
sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good 
sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly 
sudo apt-get install gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools
sudo apt-get install libgstreamer-plugins-base1.0-dev
git clone https://github.com/ros-drivers/usb_cam.git
git clone https://github.com/ros-drivers/audio_common.git
cd ..
catkin_make
