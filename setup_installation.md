Ros2 Installation Commands (Foxy):
------------------------------------------------------------
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade

sudo apt install ros-foxy-desktop

source /opt/ros/foxy/setup.bash

sudo apt install python3-rosdep2
rosdep update

sudo apt install python3-colcon-common-extensions

**Ros2 Installation Commands (Humble):**
-------------------------------------------------------------
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash

sudo apt install python3-rosdep2

rosdep update

sudo apt install python3-colcon-common-extensions

rover installation commands:
-------------------------------------------------------------
// Creating a workspace

mkdir -p ~/rover_workspace/src

cd ~/rover_workspace/src

git clone https://github.com/jackarivera/roverrobotics_mini_ros2.git

cd ..

rosdep install -i --from-path src --ignore-src

cd ..

mkdir -p library

cd library

git clone https://github.com/RoverRobotics/librover.git

cd librover

cmake .

make

sudo make install

cd ~/rover_workspace

colcon build

bluetooth (if rPi):
sudo apt install pi-bluetooth

rplidar installation commands:
--------------------------------------
cd ~/rover_workspace/src

git clone https://github.com/Slamtec/sllidar_ros2.git

cd ..

colcon build --symlink-install

sudo chmod 666 /dev/ttyUSB0

slam toolbox installation commands:
------------------------------------------
sudo apt install ros-foxy-slam-toolbox

Can Setup Commands:
-----------------------------------------------
sudo apt install can-utils

sudo ip link set can0 type can bitrate 500000 sjw 127 dbitrate 2000000 dsjw 15 berr-reporting on fd on

sudo ip link set up can0

Launch command:
-----------------------------------------------
cd ~/rover_workspace

source install/setup.sh

ros2 launch roverrobotics_driver mini_teleop.launch.py
