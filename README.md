# ROS2 Driver for Rover Robots

## About:

- This package is being created to add necessary features and improvements for our robots, specifically for ros2. Our ros2 package has been lacking in regards to out-of-the-box support for items such as URDF, Simulation, Slam, and Navigation. This package aims to bring our ros2 up to speed with all of these features.

- This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble.

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver and implement new features.

- While this driver will constantly be changing on the main branch, we have released a stable humble branch! Stable releases will be released for their respective ROS2 version (I.e. Our packages for humble and that are stable are released into the humble branch of this repository).


## Installation:
Installation is made simple through two options:

#### ``(Recommended)``Option 1: Using the provided install script in the ``install_scripts`` folder

Clone this repo: [rover_install_scripts_ros2](https://github.com/RoverRobotics/rover_install_scripts_ros2)
Then, follow the instructions in the setup script.
```
git clone https://github.com/RoverRobotics/rover_install_scripts_ros2
cd rover_install_scripts_ros2
sudo chmod 777 setup_rover.sh
./setup_rover.sh
```

This install script will ask you which robot you wish to install and additionally asks if you want to create a roverrobotics.service, setup udev rules, etc. The service automatically starts on computer boot up and runs our robot driver. If you do not wish for it to automatically start, please decline the service creation. For the mini or miti, most have a can-to-usb converter that the script will set up the drivers for. If you wish, you can also plug a micro usb into the vesc port that controls the rear right hub motor. You must also change the config file for the mini or miti to use ``comm_type: serial`` and set the corresponding ``/dev/tty*`` port.

Once the install is finished you are good to go!

#### Option 2: Manually build (No service creation, you can run setup_rover.sh and only create the service if you wish)

Clone our repository into your workspace and ``colcon build`` like any other package. Source the installation and you are ready to go. This does not create udev rules, our roverrobotics.service, or set up the can device if you are using a mitiy or mini with a can-to-usb converter. You can run the install script to do that if you wish. We **highly recommend** using the install script to perform a proper installation.

```
cd <ros2_ws>/src
git clone https://github.com/RoverRobotics/ros2_roverrobotics_development.git -b ${ROS_DISTRO}
cd ..
colcon build
source install/setup.sh
```
