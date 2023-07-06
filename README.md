# ROS2 Driver for Rover Robots

## About:

- This package is being created to add necessary features and improvements for our robots, specifically for ros2. Our ros2 package has been lacking in regards to out-of-the-box support for items such as URDF, Simulation, Slam, and Navigation. This package aims to bring our ros2 up to speed with all of these features.

- This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble.

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver and implement new features.

- This driver is very much so in the development and beta stages. Please report any issues that you face so we can create a stable release.

  

## Roadmap / To-Implement:

1) Combine librover into this repo so you only have to install one package **[DONE]**
2) Adjust librover to not repeat a ton of code. **[DONE]**
	 - Make DifferentialDrive class that has wheel radius, base length, etc as parameters to use one unified controller **[DONE]**
3) Provide better ROS based diagnostics for failures **[IN PROGRESS]**
4) URDF and Simulation support **[ADDED, Room for more improvements]**
5) Adjust control method to use VESC internal PID **[ABANDONED, Control method was not viable]**
6) Provide better status messages on status topic including battery etc **[PLANNED]**
7) Create nice configs and launch files **[IN PROGRESS]**


## Installation:
Installation is made simple through two options:

The first step is to clone the repository into a ROS2 workspace. 
```
cd <ros2_ws>/src
git clone https://github.com/RoverRobotics/ros2_roverrobotics_development.git -b ${ROS_DISTRO}
cd ..
```

#### ``(Recommended)``Option 1: Using the provided install script in the ``install_scripts`` folder

Please navigate to the ``install_scripts`` folder:

``cd install_scripts``

This script requires read/write privileges:
``sudo chmod 777 setup_rover.sh``

Finally run the install script by:
``./setup_rover.sh``

This install script will ask you which robot you wish to install and additionally asks if you want to create a roverrobotics.service for that robot. This service automatically starts on computer boot up and runs our robot driver. If you do not wish for it to automatically start, please decline the service creation.

Once the install is finished you are good to go!

#### Option 2: Manually build (No service creation, you can run setup_rover.sh and only create the service if you wish)

Clone our repository into your workspace and ``colcon build`` like any other package. Source the installation and you are ready to go. This does not create udev rules or our roverrobotics.service. You can run the install script to do that if you wish. Please note this also does not set up can drivers for the mini and miti. We highly recommend using the install script to perform a proper installation.

```
cd <ros2_ws>
colcon build
source install/setup.sh
```
