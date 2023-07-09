
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

## Usage

Source your workspace and launch your robot via:

```bash
ros2 launch roverrobotics_driver <robot>.launch.py
```
*Valid ``<robot>`` options are: ``zero, pro, mini, miti, indoor_miti``*

Alternatively,
You may launch with a teleop node which will try to connect to a joystick:
```bash
ros2 launch roverrobotics_driver <robot>_teleop.launch.py
```

## Simulation with Gazebo
Our ROS2 packages now support simulations for all robots! The ``roverrobotics_gazebo`` package implements all of the simulation launches. You can launch your simulation using the following:
```bash
ros2 launch roverrobotics_gazebo <robot>_gazebo.launch.py
```
*Valid ``<robot>`` options are: ``2wd_rover, 4wd_rover, mini, miti, indoor_miti``*

The 2wd_rover and 4wd_rover replace the Rover Zero and Rover Pro since they have the same footprint. The 2wd_rover implements our chassis with two driven front wheels and two rear casters and the 4wd_rover implements our chassis with 4 driven wheels in a skid steer configuration.

## Robot Description Setup
Our ROS2 packages now implement URDF setups for all Rover Robots! The ``roverrobotics_description`` package implements all of the URDF configs and launches. You can view a URDF using the following:
```bash
ros2 launch roverrobotics_description display_<robot>.launch.py
```
*Valid ``<robot>`` options are: ``2wd_rover, 4wd_rover, mini, miti, indoor_miti``*

The 2wd_rover and 4wd_rover replace the Rover Zero and Rover Pro since they have the same footprint. The 2wd_rover implements our chassis with two driven front wheels and two rear casters and the 4wd_rover implements our chassis with 4 driven wheels in a skid steer configuration.

**IMPORTANT:** Our launch files for the Rover Pro and Rover Zero launch the 4wd version by default. If you have a 2wd version then you **MUST** edit the launch file for the pro/zero. These launch files are found in the ``roverrobotics_driver`` package within the ``launch`` folder. At the top of the launch files there is a default_model_path. Change ``rover_4wd.urdf -> rover_2wd.urdf or flipper.urdf`` if you are using a 2wd rover or flipper, respectively.

### Transformations and Sensor Links
Transformations and Sensor Links/Frames can be easily made in the URDF file for your robot. Within the ``roverrobotics_description``  package we have provided two example sensors in the ``urdf/accessories`` folder that connect to a Rover Development Payload. You may use these as examples to create your own sensor links.

**By Default:**
Our launch files launch the robot state publisher that publishes the following transforms:
```
base_link -> chassis_link
chassis_link -> All Two/Four Wheel Links or Flipper Links
```
There are some accessories that can also be enabled. These are the example sensors. They provide the following additional transforms:
```
base_link -> chassis_link
chassis_link -> All Two/Four Wheel Links or Flipper Links
chassis_link -> payload_link
payload_link -> lidar_link
payload_link -> imu_link
```
Please view the URDF file for your robot before deploying to ensure that you have the correct links made and the sensors you want to be added to the URDF setup correctly. There are more instructions in each robots URDF file.

Additionally,
Here are some more resources for understanding transformations, urdf, and gazebo:

[(1) Gazebo Sim Docs](https://gazebosim.org/docs)

[(2) Gazebo ROS Docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

[(3) Gazebo Sim ROS Installation](https://gazebosim.org/docs/garden/ros_installation)

[(4) ROS URDF Docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

Here is an example that places a RPLidar S2 relative to the ``base_link`` instead of the payload and adds the gazebo plugin to run a lidar simulation:

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="rplidar_s2">
	<link name="lidar_link">
		<visual>
			<origin xyz="0 0 0" rpy="-1.57 0 3.1415"/>
			<geometry>
				<mesh filename="file://$(find roverrobotics_description)/meshes/rplidar_s2.dae"/>
			</geometry>
		</visual>
		<collision>
			<origin xyz="0 0 0" rpy="-1.57 0 3.1415"/>
			<geometry>
				<mesh filename="file://$(find roverrobotics_description)/meshes/rplidar_s2.dae"/>
			</geometry>
		</collision>
	</link>

	<joint name="lidar_to_payload" type="fixed">
		<parent link="base_link"/> <!-- NOTICE THE PARENT LINK IS BASE_LINK -->
		<child link="lidar_link"/>
		<origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
	</joint>
</robot>
```

## Navigation2 and Slam Toolbox
We have provided launch files and configs for Navigation2 and Slam Toolbox. They are available in the ``roverrobotics_driver`` package.

They can be launched with the following launch commands:
```
ros2 launch roverrobotics_driver slam_launch.py
ros2 launch roverrobotics_driver navigation_launch.py
```
These tools require the following transformations:
```
base_link -> odom   ## Provided by the robot_localization package
odom -> map         ## Provided by slam_toolbox
```

We highly recommend reading through the documentation for Nav2, Slam Toolbox, and Robot Localization to understand how navigation and slam works and walk through our provided configs to familiarize yourself with the concepts. Linked below are the docs for these packages.

[(1) Robot Localization Github](https://github.com/cra-ros-pkg/robot_localization) | 
[Robot Localization Tutorial by Automatic Addison](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)

[(2) Navigation2 Documentation](https://navigation.ros.org/) | 
[Navigation2 Github](https://github.com/ros-planning/navigation2)

[(3) Slam Toolbox Github and Docs](https://github.com/SteveMacenski/slam_toolbox)

We also recommend these ROS2 tutorial playlists from [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics/featured):

[(1) Getting Ready to Build with ROS](https://www.youtube.com/playlist?list=PLunhqkrRNRhYYCaSTVP-qJnyUPkTxJnBt)

[(2) Various ROS Tutorials from simulation to building a robot to software](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)

