#!/bin/sh
ROS_DISTRO=humble
LIBMPSSE_REPO=https://github.com/jackarivera/libmpsse.git
ROVER_REPO=https://github.com/RoverRobotics/ros2_roverrobotics_development.git

# Installs the necessary components
echo "Installing dependencies.."
sudo apt install git -y
sudo apt install can-utils net-tools openssh-server -y
sudo apt install swig libftdi-dev -y
echo "Dependencies installed."


# Download and install libmpsse
echo "Installing libmpsse dependency.."
cd ~/Documents
git clone $LIBMPSSE_REPO
cd src
./configure --disable-python
make
make install
echo "Installed libmpsse."

# ROS Packages
echo "Installing ROS packages.."
sudo apt install ros-$ROS_DISTRO-slam-toolbox -y
sudo apt install ros-$ROS_DISTRO-navigation2 -y
sudo apt install ros-$ROS_DISTRO-nav2-bringup -y
sudo apt install ros-$ROS_DISTRO-imu-tools -y
echo "Installed ROS packages."

# Install rover driver
echo "Cloning Rover Robotics driver.."
cd ~/
mkdir rover_workspace
cd rover_workspace
mkdir src
cd src
git clone $ROVER_REPO -b $ROS_DISTRO

echo "Building Rover Robotics driver.."
cd ~/rover_workspace
colcon build
