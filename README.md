
# ROS2 Driver for Rover Robots

## About:

- This package is being designed to run on our new carrier board but will maintain functionality with all previous robots that do not have a carrier board.

- This package is exclusively built for ROS2. It is being tested on Ubuntu 20.04 (Docker?) & 22.04 with ROS2-Humble.

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver.

- This driver is very much so in the development and beta stages. Please report any issues that you face so we can create a stable release.

  

## Roadmap / To-Implement:

1) Combine librover into this repo so you only have to install one package
2) Adjust librover to not repeat a ton of code.
	 - Make DifferentialDrive class that has wheel radius, base length, etc as parameters to use one unified controller
3) Provide better ROS based diagnostics for failures
4) URDF and Simulation support
5) Adjust control method to use VESC internal PID
6) Provide better status messages on status topic including battery etc
7) Create nice configs and launch files

  
