#!/usr/bin/env bash

# install git, download repo
sudo apt-get install git
git clone https://github.com/haelannaleah/turtlebot_time.git

# configure apt-get repos to accept everything (first back up current sources)
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
sudo add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu trusty restricted universe multiverse"
sudo add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu trusty-updates restricted universe multiverse"
sudo apt-get update

# set up sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# set up keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

# install ros
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

# install specific ros packages
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs

# set up custom packages
echo "source /catkin_workspace/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc