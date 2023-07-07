#!/bin/sh
ROS_DISTRO=humble
ROVER_REPO=https://github.com/RoverRobotics/ros2_roverrobotics_development.git

# Define the install_ros_packages function
install_ros_packages() {
    echo "Installing ROS packages.."
    sudo apt-get install -y ros-$ROS_DISTRO-slam-toolbox > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error encountered while installing ros-$ROS_DISTRO-slam-toolbox. Exiting..."
        exit 1
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-navigation2 > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error encountered while installing ros-$ROS_DISTRO-navigation2. Exiting..."
        exit 1
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-nav2-bringup > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error encountered while installing ros-$ROS_DISTRO-nav2-bringup. Exiting..."
        exit 1
    fi
    sudo apt-get install -y ros-$ROS_DISTRO-imu-tools > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error encountered while installing ros-$ROS_DISTRO-imu-tools. Exiting..."
        exit 1
    fi
    echo "Finished installing ROS packages successfully."
}

clear

# Prompt the user for device type
printf "Enter the robot type (indoor_miti, miti, mini, zero, pro): "
read device_type

# Check if the entered device type is valid
if [ "$device_type" != "indoor_miti" ] && [ "$device_type" != "miti" ] && [ "$device_type" != "mini" ] && [ "$device_type" != "zero" ] && [ "$device_type" != "pro" ]; then
    echo "Invalid robot type. Exiting..."
    exit 1
fi

# Prompt the user to decide about installing automatic start service
while true; do
    printf "Would you like to install the automatic start service? [y/n]: "
    read install_service
    case $install_service in
        [Yy]* ) install_service=true; break;;
        [Nn]* ) install_service=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done


# Prompt the user to decide about installing the udev rules
while true; do
    printf "Would you like to install the udev rules? [y/n]: "
    read install_udev
    case $install_udev in
        [Yy]* ) install_udev=true; break;;
        [Nn]* ) install_udev=false; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

clear

echo "====================================="
echo "                                     "
echo "Installation settings:               "
echo "----------------------------         "
echo "Robot type:   $device_type           "
echo "Service:      $install_service       "
echo "Udev:         $install_udev          "
echo "                                     "
echo "====================================="
echo ""

# ROS Packages
install_ros_packages

if [ "$install_service" = true ]; then
    echo "Installing the automatic start service..."
    # Insert your command for installing the service here
    # your_command > /dev/null | Silences standard messages but shows errors
fi

if [ "$install_udev" = true ]; then
    echo "Installing the udev rules..."
    # Insert your command for installing the udev rules here
fi


# # Install rover driver
# echo "Cloning Rover Robotics driver.."
# cd ~/
# mkdir rover_workspace
# cd rover_workspace
# mkdir src
# cd src
# git clone $ROVER_REPO -b $ROS_DISTRO

# echo "Building Rover Robotics driver.."
# cd ~/rover_workspace
# colcon build
