# ROS2 Driver for Rover Robots
## About:
- This package is being designed to run on our new carrier board but will maintain functionality with all previous robots that do not have a carrier board.
- This package is exclusively built for ROS2. It is being tested on Ubuntu 20.04 (Docker?) & 22.04 with ROS2-Humble.
- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver.
- This driver is very much so in the development and beta stages. Please report any issues that you face so we can create a stable release.

## Roadmap / To-Implement:
*Note these are in no particular order. Please look at the priority/in-progress tags on each to see if they are being worked on. H = High, M = Medium, L = Low*

1) Restructuring of Launch files **(Priority - H, In Progress)**
	- Our goal is to make it more organized and easier to launch various packages
2) Out of the box SLAM and Nav2 **(Priority - M, In Progress)**
	- With autonomous robotics growing in popularity we feel it is important to be able to provide a out-of-the-box solution so our customers can open the box and have SLAM and Nav2 running in no time
3) Foxglove implementation ([Foxglove](https://foxglove.dev/)) **(Priority H, In Progress)**
	- We have found foxglove to be a very useful tool while developing. In order to provide this out-of-the-box to our customers we will be implementing an automated launch (if chosen on install) of foxglove_bridge so you can easily connect and visualize your robot data.
4) Better diagnostics **(Priority - M)**
	- When I came onto rover, it was very hard for me to diagnose issues with the robots.
	- My goal is to provide thorough diagnostics that make it easy to tackle any problem that arises.
5) New controller driver **(Priority - M)**
	- Our current controller driver presents some issues when running over can causing jittering. A new controller driver / fixed driver is in the works.
6) URDF Meshes and Simulation Support **(Priority - L)**
	- Simulation is an important part of robotics prototyping and we aim to provide the tools necessary to get up and running quicker and easier
	
7) AprilTag/Fiducial Tracking **(Priority - L)** and Camera support **(Priority - M)**
	- We currently offer several cameras for sale with our robots, thus we should also offer turnkey solutions for these devices. I'm aiming to add camera driver support out-of-box as well as the ability to track apriltags/fiducials as the user needs.
8) Updated documentation and tutorials **(Priority - H, In Progress)**
	- We want our robots to be used for both research and education. It is important that we can provide accurate, up-to-date, and easy to follow documentation to help our users get started and understand the different aspects of ROS and our robots.
9) New Install Script **(Priority - H, In Progress)** ([Rover Installer](https://github.com/jackarivera/rover_install))
	- Our current script, to put it bluntly, is lackluster. The new script will be much easier to work with and easier to debug IF NEEDED.

## Planned fixes for Issues:
1) Communication Issues on Zero3
	- Zero3 communication seems to have issues with the ros2 driver. Unfortunately until about may I won't really be able to work on this as I don't have access to a zero3 currently. I will work on fixing this in this new driver as soon as may comes.

## Completed Features:
Once projects leave the roadmap, they will be placed here. This is mainly for me to stay organized but it also allows you to see what is currently done. If any bugs make it through to here please post an issue and I will try to stay on top of them.

## Installation instructions - Outdated (Do Not Follow)

1. Cloning this repository into your workspace
```
cd workspace/src/
git clone https://github.com/RoverRobotics/roverrobotics_ros2.git
```
2. Install Udev rules for robot
```
cd workspace/src/roverrobotics_driver/udev
sudo cp 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules && sudo udevadm control --reload-rules && udevadm trigger
```
3. Install shared library
``` 
cd ~/
mkdir library/
cd library/
git clone https://github.com/RoverRobotics/librover
cd librover/
cmake .
make
sudo make install 
```
4. Rebuild your workspace
```
cd workspace/
colcon build
```
5. Update env variables and configuration files 
```
source install/setup.sh
```
6. Launch Robot (replace <launch file name> with your robot config.)
```
ros2 launch roverrobotics_driver <launch file name>
```
  ```
  Launch Files:
  <model>.launch.py: Launches robot configuration for specific rover robot. (i.e. Mini, Mega, Pro2, Zero2)
  <model>_teleop.launch.py: Launches robot configuration with teleop controller enabled.
  rover_slam_mapping.launch.py: Launches lidar and slam toolbox in asynchronous mapping mode(Requires slam package).
  rover_slam_localization.launch.py: Launches lidar and slam toolbox in localization mapping mode.
  ```
