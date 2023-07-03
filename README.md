
# ROS2 Driver for Rover Robots

## About:

- This package is being created to add necessary features and improvements for our robots, specifically for ros2. Our ros2 package has been lacking in regards to out-of-the-box support for items such as URDF, Simulation, Slam, and Navigation. This package aims to bring our ros2 up to speed with all of these features.

- This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble.

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver and implement new features.

- This driver is very much so in the development and beta stages. Please report any issues that you face so we can create a stable release.

  

## Roadmap / To-Implement:

1) Combine librover into this repo so you only have to install one package **DONE**
2) Adjust librover to not repeat a ton of code. **DONE** (Still not pretty but better)
	 - Make DifferentialDrive class that has wheel radius, base length, etc as parameters to use one unified controller **DONE**
3) Provide better ROS based diagnostics for failures **IN PROGRESS**
4) URDF and Simulation support **ADDED, Room for more improvements**
5) Adjust control method to use VESC internal PID **ABANDONED, Control method was not viable**
6) Provide better status messages on status topic including battery etc **PLANNED**
7) Create nice configs and launch files **IN PROGRESS**
