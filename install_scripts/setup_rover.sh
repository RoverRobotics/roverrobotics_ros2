#!/bin/sh
ROS_DISTRO=humble
ROVER_REPO=https://github.com/RoverRobotics/ros2_roverrobotics_development.git


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
