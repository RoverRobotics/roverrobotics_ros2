
# ROS 2 Driver for Rover Robots

  

## About:

  

- This package is being created to add necessary features and improvements for our robots, specifically for ros2. Our ros2 package has been lacking in regards to out-of-the-box support for items such as URDF, Simulation, Slam, and Navigation. This package aims to bring our ros2 up to speed with all of these features.

  

- This package is exclusively built for ROS2. The ``jazzy`` branch is tested on Ubuntu 24.04 with ROS 2 Jazzy; the ``humble`` branch is tested on Ubuntu 22.04 with ROS 2 Humble.

  

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver and implement new features.

  

- Stable releases are published on a branch per ROS 2 distribution. Use the branch matching your distribution (``humble`` or ``jazzy``) rather than the development branch.

  
  

## Installation:

Installation is made simple through two options:

  

#### ``(Recommended)``Option 1: Using the provided install script in the ``rover_install_scripts_ros2`` repo

  

Clone this repo: [rover_install_scripts_ros2](https://github.com/RoverRobotics/rover_install_scripts_ros2)

Then, follow the instructions in the setup script.

```

git clone https://github.com/RoverRobotics/rover_install_scripts_ros2

cd rover_install_scripts_ros2

bash setup_rover.sh

```

  

This install script will ask you which robot you wish to install and additionally asks if you want to create a roverrobotics.service, setup udev rules, etc. The service automatically starts on computer boot up and runs our robot driver. If you do not wish for it to automatically start, please decline the service creation. For the mini or miti, most have a can-to-usb converter that the script will set up the drivers for. If you wish, you can also plug a micro usb into the vesc port that controls the rear right hub motor. You must also change the config file for the mini or miti to use ``comm_type: serial`` and set the corresponding ``/dev/tty*`` port.

  

Once the install is finished you are good to go!

  

#### Option 2: Manually build (No service creation, you can run setup_rover.sh and only create the service if you wish)

  

Clone our repository into your workspace and ``colcon build`` like any other package. Source the installation and you are ready to go. This does not create udev rules, our roverrobotics.service, or set up the can device if you are using a mitiy or mini with a can-to-usb converter. You can run the install script to do that if you wish. We **highly recommend** using the install script to perform a proper installation.

  

```

cd <ros2_ws>/src

git clone https://github.com/RoverRobotics/roverrobotics_ros2.git -b ${ROS_DISTRO}

cd ..

colcon build

source install/setup.sh

```

## Usage

Source your workspace and launch your robot via:

```bash
ros2 launch roverrobotics_driver <robot>.launch.py
```
*Valid ``<robot>`` options are: ``zero, pro, mini, mini_2wd, miti, miti_65, max, mega``*

Alternatively,
You may launch with a teleop node which will try to connect to a joystick:
```bash
ros2 launch roverrobotics_driver <robot>_teleop.launch.py
```

### What is launched with this?
Our launch files launch (1) The Robot Driver, (2) The robot description, (3) an accessories launch, and (4) A PS5 Controller Driver.
(1) The Robot Driver: responsible for interfacing with our robot and handling velocity commands as well as publishing wheel odometry
(2) The Robot Description: responsible for publishing to the /robot_description topic and providing transforms between the base_link, chassis_link, and payload_link. Edit the URDF for your robot to define new frames or remove links
(3) Accessories Launch: a convenience launch for sensor packages to run when the robot is launched. 
(4) PS5 Controller Driver: handles input from the PS5 (DualSense) Controller

The teleop launch files use ``ps5_controller.launch.py`` by default. To use a PS4 controller instead, either pass ``--gamepad ps4`` to ``setup_rover.sh``, or edit the robot's ``*_teleop.launch.py`` to include ``ps4_controller.launch.py``. Button and axis mappings live in ``config/ps4_controller_config.yaml`` and ``config/ps5_controller_config.yaml``; the ``*_jp6`` variants carry the mapping used on JetPack 6 and newer Jetson images, where the pad enumerates differently.

#### Controller buttons

| Control | Action |
| --- | --- |
| Left stick, vertical | Forward and reverse speed |
| Right stick, horizontal | Turn rate |
| D-pad up / down | Raise / lower the linear speed scale |
| D-pad left / right | Raise / lower the turn speed scale |
| **Circle (○)** | **Software emergency stop.** Latches the robot stopped until reset. |
| **Triangle (△)** | **Reset the emergency stop.** The robot resumes on the next stick input. |

The estop buttons work on both the PS4 and the PS5 controller. They are defined in ``config/topics.yaml`` by button name (``B`` for Circle, ``Y`` for Triangle), and every controller config maps those names to the correct physical buttons for its kernel driver, so no per-controller change is needed. A single press sends a single message; holding the button does not repeat it.


## Driver Configuration

Each robot has a config file in ``roverrobotics_driver/config`` (for example ``mega_config.yaml``) that is loaded by its launch file. The parameters below control the drivetrain.

### Connection

**CAN robots require the ``rovercan`` udev rule.** Kernel CAN numbering is not stable: a host with onboard CAN controllers claims ``can0`` through ``can3``, so a USB-CAN adapter can enumerate as ``can4`` on one boot and something else on the next. A hardcoded ``can0`` then configures an onboard controller that has nothing attached, and the driver reports a healthy connection on a bus that carries no traffic. The udev rule pins the adapter to the fixed name ``rovercan`` regardless of enumeration order.

``setup_rover.sh`` installs the rule. If you build manually (Option 2 above), install it yourself before running the driver, or the interface will not exist:

```bash
cd ~/rover_install_scripts_ros2/udev
sudo cp 99-can-usb.rules /etc/udev/rules.d/99-can-usb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Confirm with ``ip -br link``, which should list ``rovercan``.

| Parameter | Description |
| --- | --- |
| ``robot_type`` | Robot model: ``zero``, ``pro``, ``mini``, ``mini_2wd``, ``miti``, ``max``, ``mega``. |
| ``comm_type`` | ``can`` or ``serial``. |
| ``device_port`` | CAN interface or serial device. All CAN configs ship with ``rovercan``, the persistent name given to the USB-CAN adapter by the udev rule that ``setup_rover.sh`` installs. See the note below if you are building manually. |

### Identification

| Parameter | Description |
| --- | --- |
| ``serial_number`` | Free text identifying this unit. Published latched on ``rover_<robot_type>/serial_number``. Any format works: digits, letters, dashes, any length. Defaults to empty, in which case the driver logs a warning at startup. |

### Kinematics

| Parameter | Description |
| --- | --- |
| ``wheel_radius`` | Wheel radius in meters. Must match the fitted wheel, as it scales both odometry and commanded velocity. |
| ``wheel_base`` | Distance between left and right wheel centers, in meters. |
| ``robot_length`` | Distance between front and rear wheel centers, in meters. |
| ``motor_pole_pairs`` | Pole pairs in the drive motor, i.e. half the pole count. A VESC reports electrical RPM, and mechanical RPM is electrical RPM divided by this. **Mini and MITI are 30-pole, so 15.0; MAX and MEGA are 20-pole, so 10.0.** Getting it wrong scales every wheel speed and all of wheel odometry by the same factor. Write it as a decimal (``15.0``), not an integer. |
| ``gear_ratio`` | Motor revolutions per wheel revolution on geared drivetrains. Applied to the RPM feedback to convert motor RPM to wheel RPM. The MEGA and MAX are geared; the Mini and MITI are direct drive and must stay at ``1.0``, which makes the conversion a no-op. Values of zero or less are rejected and treated as ``1.0``. |

The MAX is supported with 13 inch (``max_130_config.yaml``, radius 0.1651) and 15 inch (``max_150_config.yaml``, radius 0.1905) wheels. The 6.5 inch and 10 inch variants are no longer supported and their configs and URDFs have been removed. Selecting the wrong config silently scales odometry and commanded velocity, so confirm the radius matches the wheels actually fitted.

### Wheel trim

| Parameter | Description |
| --- | --- |
| ``wheel_trim_fl`` | Scale factor applied to the front-left wheel target speed. |
| ``wheel_trim_fr`` | Front-right scale factor. |
| ``wheel_trim_rl`` | Rear-left scale factor. |
| ``wheel_trim_rr`` | Rear-right scale factor. |

All four default to ``1.0``. Use them to compensate for a wheel that runs fast or slow relative to the others, for example after replacing a single motor. Reduce the fast wheel rather than increasing the slow one, so the robot keeps its commanded top speed.

### Velocity handling

| Parameter | Description |
| --- | --- |
| ``max_velocity_step`` | Maximum reduction in linear velocity per 50 ms cycle, applied to forward motion only. Limits how sharply the robot decelerates when a command drops. **0.75 on every robot**, chosen by testing on hardware: the earlier 0.05 made the robot coast noticeably after the stick was released. On the CAN robots the motor library also caps acceleration at 5 m/s^2 relative to the measured speed, and that cap is the one in effect at any value above about 0.25, so small changes here have no effect. |
| ``cmd_vel_timeout_sec`` | If no new message arrives on the velocity topic within this period, the velocity targets are zeroed and the robot ramps to a stop. Defaults to ``0.3``. |

The timeout is a safety stop for a lost or stalled publisher. Any node commanding the robot must publish continuously, not once per change of speed.

The timeout is measured on a monotonic clock, so a wall-clock jump (for example NTP correcting the time shortly after boot) can neither trigger it falsely nor disable it. Velocity commands containing ``NaN`` or infinity are rejected and logged, and do not refresh the timeout. A command smaller than 0.001 in both ``linear.x`` and ``angular.z`` is treated as a stop, so a planner that publishes a tiny residual such as ``1e-9`` still brings the robot fully to rest.

### Stopping and braking

These parameters shape how a CAN robot comes to rest when the commanded speed drops. The braking band is off unless a config enables it, so a robot without these keys behaves exactly as before.

| Parameter | Description |
| --- | --- |
| ``rest_wheel_rpm`` | Below this wheel speed, a wheel on a commanded stop is released: its duty is zeroed and its PID reset, so leftover duty cannot rock it back and forth. Default ``8.0``. Accepted range (0, 30]. |
| ``brake_band_duty`` | Enables the braking band (``0.0`` = off). While a wheel is rolling faster than its target, its duty is not allowed to fall more than this far below the duty that would hold its current speed. This bounds the braking current, and with it the energy pushed back into the battery. Valid 0.0 to 0.5. |
| ``brake_band_rpm`` | Above this wheel speed the band narrows in proportion to 1/rpm, so braking current stays roughly constant as speed rises instead of growing with it. |
| ``rpm_per_duty`` | Wheel rpm produced by one unit of duty with no load, i.e. the motor's back-EMF constant expressed at the wheel. The band is computed from it. With the band enabled, values outside 300 to 340 are rejected as a likely typo and the band is switched off. |
| ``release_hold_s`` | Optional. Keeps the motors actively braked for this many seconds after the wheels read zero before releasing them, which can help hold the robot on a slope. ``0.0`` (default) releases immediately. Accepted range 0 to 1. |

**Why the braking band exists.** The wheel controller accumulates duty and lowers it gradually on a stop. Without the band it overshoots through zero and briefly commands the opposite direction while the wheel is still turning, which on a VESC means plugging the motor: the robot stops with a sharp kick and can rock. At high speed the same controller can brake hard enough to push the battery's regenerative current past what its protection circuit accepts, which opens the charge path and lets the bus voltage climb far above the pack voltage. The band removes both: duty lands on zero at low speed and the motor's own short-circuit brake finishes the stop, and braking current is capped at speed.

**Shipped values.** Only the MAX configs enable the band:

```yaml
    rest_wheel_rpm: 8.0
    brake_band_duty: 0.10
    brake_band_rpm: 160.0
    rpm_per_duty: 325.0
    release_hold_s: 0.0
```

**Calibrating ``rpm_per_duty``.** Put the robot on a stand with the wheels clear of the ground, drive it at three or four steady speeds (for example 1, 2, 3 and 3.8 m/s) and read the wheel rpm and duty from the VESC status frames. Divide rpm by duty at each speed and use the value measured at the higher speeds. On the MAX 130 this measured 322 to 329, so 325 is used. Stay within 320 to 335 unless you have measured otherwise: higher values brake harder and can trip the battery protection, lower values brake more softly and can let the robot roll further downhill.

The band applies in ``INDEPENDENT_WHEEL`` mode only. It does not act during an emergency stop, which always brakes as hard as the motors allow.

### Diagnostics

| Parameter | Description |
| --- | --- |
| ``odometry_frequency`` | Publish rate for the wheel odometry topic, in Hz. |
| ``linear_covariance`` / ``yaw_covariance`` | Uncertainty published on the odometry **twist**, i.e. the measured velocity. |
| ``pose_linear_covariance`` / ``pose_yaw_covariance`` | Uncertainty published on the odometry **pose**. The pose is dead reckoned from wheel rotation, so it drifts and these must not be zero. A fusion node reads an all-zero covariance as "no uncertainty" and will trust wheel odometry over every other sensor. |
| ``robot_status_frequency`` | Publish rate for the robot status topic, in Hz. |
| ``motor_control_p_gain`` | On the CAN robots the controller output is **added** to the previous duty every cycle, so this gain behaves as the integral term: it sets how quickly duty ramps toward the target and how small the steady-state speed error is. |
| ``motor_control_i_gain`` | Leave at zero on the CAN robots. Because the output is accumulated, any value here acts as a double integrator. The remaining speed error (about 2% on the MITI) comes from a fixed 0.989 per-cycle decay on the accumulated duty. |
| ``motor_control_d_gain`` | On the CAN robots this behaves as the proportional term, and provides the damping. Too low and the wheels overshoot and ring after a speed change; too high and turning in place goes unstable. |

Retune the gains if you change ``motor_pole_pairs``, ``gear_ratio``, wheel size or motors. The controller's feedback is the measured wheel speed, so anything that changes that number changes the effective loop gain.

Shipped gains, tuned on hardware:

| Robot | ``p_gain`` | ``i_gain`` | ``d_gain`` | Notes |
| --- | --- | --- | --- | --- |
| Mini | 0.0008 | 0.0 | 0.00006 | Tuned on a Mini, Orin Nano, ROS 2 Jazzy |
| MITI | 0.0007 | 0.0 | 0.00009 | |
| MAX 130 | 0.0012 | 0.0 | 0.00006 | Tuned with a 50 to 70 lb payload |
| MAX 150 | 0.0012 | 0.0 | 0.00006 | Same drivetrain as the MAX 130; verify on the first unit |
| MEGA | 0.0012 | 0.0 | 0.000005 | |


### Control mode

| Parameter | Description |
| --- | --- |
| ``control_mode`` | ``INDEPENDENT_WHEEL`` (also accepted as ``closed_loop``) runs the per-wheel PID against measured wheel speed and is the default. ``TRACTION_CONTROL`` runs one PID per side and cuts power to the faster wheel on that side; it is experimental, and on a MITI it doubled speed ripple when turning in place, so it is not recommended. On the Mini, MITI, MAX and MEGA any other value is treated as ``INDEPENDENT_WHEEL``, because ``OPEN_LOOP`` would command full duty on those robots; the driver logs a warning. The mode in use is logged at startup. |

### Speed limits and steering response

| Parameter | Description |
| --- | --- |
| ``linear_top_speed`` | Upper bound on commanded forward speed, m/s. |
| ``angular_top_speed`` | Upper bound on commanded yaw rate, rad/s. |
| ``angular_a_coef``, ``angular_b_coef``, ``angular_c_coef`` | Coefficients of a quadratic that scales the commanded yaw rate by the robot's measured forward speed: ``scale = a*v^2 + b*v + c``. Use it to reduce steering authority as the robot speeds up. All three default to 0. |
| ``angular_min_scale``, ``angular_max_scale`` | Clamp on that scale factor. Both default to 1.0, which makes the quadratic inert: **leave them at 1.0 unless you are deliberately tuning steering response**, because with the coefficients at 0 the raw scale would otherwise clamp to zero and the robot would not turn. |

### Topics and frames

| Parameter | Description |
| --- | --- |
| ``speed_topic`` | Velocity command topic the driver subscribes to. Defaults to ``/cmd_vel/managed``; the shipped configs set ``/cmd_vel``. |
| ``odom_topic`` | Odometry publish topic, ``/odometry/wheels`` on the shipped configs. |
| ``odom_frame_id`` / ``odom_child_frame_id`` | Frame names in the odometry message, normally ``odom`` and ``base_link``. |
| ``publish_tf`` | Whether the driver broadcasts the odom to base_link transform. **False on every shipped config**, because the transform is normally published by a localisation node fusing wheel odometry with other sensors. Setting it true while such a node runs gives two publishers of the same transform. |
| ``robot_status_topic`` / ``robot_info_topic`` | Where the status and info arrays are published. |
| ``trim_topic`` | Topic for runtime trim events, ``/trim_event``. |

### Emergency stop

| Parameter | Description |
| --- | --- |
| ``estop_trigger_topic`` | A ``std_msgs/Bool`` of ``true`` here latches the robot stopped. |
| ``estop_reset_topic`` | A ``std_msgs/Bool`` of ``true`` here clears it. |
| ``estop_state`` | Initial state at startup. False on every shipped config. |

**How the emergency stop behaves.** When the trigger arrives, the next control cycle (30 ms) sends a duty of zero to every VESC, which short-circuits the motors and brakes as hard as they allow. Stick and ``cmd_vel`` input is ignored while the stop is latched. The wheel controller's accumulated duty, PIDs and braking state are cleared, so when the stop is reset the robot resumes from rest with no jump; it moves again only when a new, non-zero command arrives.

From the controller, press **Circle (○)** to stop and **Triangle (△)** to reset. From a terminal:

```bash
ros2 topic pub --once /soft_estop/trigger std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /soft_estop/reset   std_msgs/msg/Bool "{data: true}"
```

Measured on a loaded MAX 130: from 2.4 m/s the robot stops in about 0.65 s; from full speed (about 4.6 m/s) in about 1.4 s. An emergency stop at full speed returns a large amount of energy to the battery and briefly raised the bus to about 55 V in testing, so do not use it as the routine way of stopping at top speed.

## Published Topics and Units

All values are SI, following standard ROS message conventions. The message types carry no unit metadata, so they are listed here.

### `/joint_states` (`sensor_msgs/JointState`)

One entry per driven wheel, named after the URDF joints ``fl_wheel_to_chassis``, ``fr_wheel_to_chassis``, ``rl_wheel_to_chassis``, ``rr_wheel_to_chassis``. Published at ``odometry_frequency``.

| field | unit | meaning |
| --- | --- | --- |
| ``position`` | radians | Cumulative angle that wheel has rotated since the driver started. It is not wrapped to one revolution, and it resets when the node restarts. Multiply by ``wheel_radius`` for distance that wheel has covered. |
| ``velocity`` | radians/second | Current rotational speed of that wheel. Multiply by ``wheel_radius`` for that wheel's ground speed in m/s. |

``position`` is integrated from ``velocity`` inside the driver rather than read from the VESC tachometer, so it accumulates drift and any lost CAN frame is lost distance. Use ``velocity`` when you want an instantaneous measurement, for example when balancing wheel trims.

Only robots with four driven wheels publish this (Mini, MITI, MAX, MEGA). The Rover Pro, Zero and Mini 2WD have different joints in their URDFs and are not covered.

### `/odometry/wheels` (`nav_msgs/Odometry`)

| field | unit |
| --- | --- |
| ``pose.pose.position`` | metres |
| ``pose.pose.orientation`` | quaternion |
| ``twist.twist.linear.x`` | metres/second |
| ``twist.twist.angular.z`` | radians/second |

The pose is integrated from the **instantaneous** wheel velocity. The rolling mean is used only for the published twist, because integrating a smoothed velocity makes the pose lag the robot by half the averaging window and turns every acceleration into position error.

Yaw is wrapped to [-pi, pi] rather than accumulating without bound.

Covariance is a row-major 6x6 over (x, y, z, roll, pitch, yaw), so the diagonal is at indices 0, 7, 14, 21, 28, 35. Pose x/y/yaw and twist vx/vyaw come from the parameters above. Lateral velocity is structurally zero on a differential drive, so its covariance is near zero rather than being given the forward-velocity value. z, roll and pitch are not observable on a planar drive and are published as 1e6 so a fusion node ignores them instead of trusting a zero.

**The pose is not corrected by anything.** It drifts, and on a skid-steer the yaw drifts fastest because turning requires the wheels to scrape. Fuse it with an IMU or a scan matcher for anything that needs absolute position.

#### Resetting the pose

```bash
ros2 topic pub --once /roverrobotics_driver/reset_odometry std_msgs/msg/Empty "{}"
```

Zeroes x, y and yaw without restarting the driver. Useful at the start of a measured test run.

### `/cmd_vel` (`geometry_msgs/Twist`, subscribed)

| field | unit | meaning |
| --- | --- | --- |
| ``linear.x`` | metres/second | Forward speed. Positive is forward. |
| ``angular.z`` | radians/second | Yaw rate. Positive is counter-clockwise (turning left). |

``linear.y``, ``linear.z``, ``angular.x`` and ``angular.y`` are ignored on a differential drive.

The teleop ceiling comes from the controller config, not the driver. In ``ps4_controller_config.yaml`` and ``ps5_controller_config.yaml`` the ``scale`` entry sets the value at full stick deflection, so ``LEFT_JOY_VERT: scale: 1.25`` means full forward stick publishes ``linear.x = 1.25`` m/s, and ``RIGHT_JOY_HORIZ: scale: 2.5`` means full sideways stick publishes ``angular.z = 2.5`` rad/s. Raise or lower those to change the robot's top teleop speed.

### `rover_<robot_type>/battery_status` (`sensor_msgs/BatteryState`)

| field | unit | notes |
| --- | --- | --- |
| ``voltage`` | volts | bus voltage measured at the motor controller |
| ``current`` | amps | **not a battery current; do not use.** See the note below |
| ``percentage`` | **percent, 0 to 100** | estimated from voltage; see the notes below |
| ``present`` | bool | true whenever the driver has a live connection to the robot |

``header.stamp`` is set from the node clock on every publish.

**``percentage`` is deliberately 0 to 100, not the 0 to 1 that the message definition specifies.** Rover Robotics publishes the state of charge directly because it is what customers already read and it is easier to interpret in a terminal. A generic battery widget that assumes the ROS convention will therefore read it 100 times too high. Divide by 100 if you are feeding a tool that expects the standard range.

**``current`` does not measure the battery.** The robots have no pack current sensor. The value is the input current reported by one motor controller (the VESC with CAN ID 1, the only one that sends its input-current status), so it covers one of four motors and excludes the other three, the computer and accessories. It cannot detect charging: the only negative input current a VESC sees is its own motor regenerating while braking. In addition, the driver currently decodes this field incorrectly (as unsigned, with the wrong scale), so the published number is not meaningful. Until that is fixed, ignore ``current``.

**``percentage`` is an estimate from voltage,** mapped linearly from 34 V (0%) to 42 V (100%) with no load compensation. It drops while the motors draw current and rises again at rest, so read it with the robot idle.

Fields the VESCs do not report are left at their defaults: ``temperature``, ``charge``, ``capacity``, ``design_capacity``, ``power_supply_status``, ``power_supply_health``, ``power_supply_technology``, ``location``, ``cell_voltage`` and ``cell_temperature``.

### `rover_<robot_type>/serial_number` (`std_msgs/String`)

The unit's serial number, taken from the ``serial_number`` parameter in the robot config. It is free text, so any scheme works: digits, letters, dashes, any length.

Published **once at startup with transient-local (latched) durability**, so a subscriber that starts later still receives it without the driver republishing. Read it with:

```bash
ros2 topic echo /rover_miti/serial_number --once
```

The parameter defaults to an empty string. When it is unset the driver still publishes, and logs a warning at startup so a missing serial is visible rather than silent. To set it, edit the robot's config file and restart the driver:

```yaml
roverrobotics_driver:
  ros__parameters:
    serial_number: "MITI-2024-0042"
```

### `/robot_info` (`std_msgs/Float32MultiArray`)

Five values: robot GUID, firmware version, speed limit, fan speed and fault flag.

Two caveats. It is **not published periodically** - it publishes only in response to a request, and the request subscriber is currently bound to the estop reset topic rather than to ``robot_info_request_topic``. And because the message is ``Float32MultiArray``, any value above 16,777,216 loses precision silently, so it is not a suitable place for a serial number. Use ``serial_number`` above instead.

### `/robot_status` (`std_msgs/Float32MultiArray`)

A flat, unlabelled array. Indices 0-19 are five values per motor in the order id, rpm, current, temperature, MOSFET temperature, for motors 1 to 4. So per-wheel RPM sits at indices 1, 6, 11 and 16. RPM here is **wheel** RPM, after the pole-pair and gear-ratio conversion. Prefer ``/joint_states`` for per-wheel work.


### Timestamps

Every message type that has a header carries a stamp set from the node clock: ``/odometry/wheels``, ``/joint_states``, ``rover_<type>/battery_status`` and the ``odom`` transform.

``/robot_status``, ``/robot_info`` and ``rover_<type>/serial_number`` use ``std_msgs`` types, which have **no header field at all**, so they cannot carry a timestamp. A timestamp also cannot be smuggled into the ``Float32MultiArray`` payload, because a ROS epoch time needs 31 bits and ``float32`` holds only 24 bits of mantissa, so the value would be silently rounded. Giving those topics a stamp would mean changing their message types.

Per-wheel RPM and battery state are both available on stamped topics already: use ``/joint_states`` and ``rover_<type>/battery_status`` when you need to correlate readings in time.

## Troubleshooting

**The robot stops responding a few seconds after the driver starts, or never responds after boot.** If ``/joy`` and ``/cmd_vel`` stop reaching the driver roughly 20 to 30 seconds after every start, while the controller stays connected, the ROS 2 middleware has stopped delivering messages between processes on the robot. On a JetPack 5 Orin Nano this was traced to Fast DDS's shared-memory transport. Switching the robot to Cyclone DDS fixes it:

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

Also add ``export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` near the top of ``/usr/sbin/roverrobotics`` so the service uses it, then restart the service. Every process that talks to the robot, including any diagnostic script, should use the same middleware.

**A new controller will not pair.** If a Bluetooth scan finds no devices at all, reset the adapter and scan again:

```bash
sudo systemctl restart bluetooth
sudo hciconfig hci0 reset
```

A PS4 controller must reconnect itself: after pairing, press its PS button rather than connecting from the robot.

## Simulation with Gazebo
Our ROS2 packages now support simulations for all robots! The ``roverrobotics_gazebo`` package implements all of the simulation launches. You can launch your simulation using the following:
```bash
ros2 launch roverrobotics_gazebo <robot>_gazebo.launch.py
```
*Valid ``<robot>`` options are: ``2wd_rover, 4wd_rover, flipper, mini, mini_2wd, miti, miti_65, max, mega``*

The 2wd_rover and 4wd_rover replace the Rover Zero and Rover Pro since they have the same footprint. The 2wd_rover implements our chassis with two driven front wheels and two rear casters and the 4wd_rover implements our chassis with 4 driven wheels in a skid steer configuration.

Note: You have to install gazebo specifically for ROS. Our install script does not install gazebo. To install gazebo:
```sudo apt install ros-{DISTRO}-ros-gz```

## Getting the Sensor Packages
At rover we have several mainly used sensors that we use. The BNO055 IMU and RP Lidar S2 are our goto IMU and Lidar sensors. Our install script does not automatically install these packages as not everyone needs them. To install them, follow the steps mentioned below to download the packages for BNO055 IMU and Slamtec RPLIDAR S2:
```bash
cd rover_workspace/src
git clone https://github.com/flynneva/bno055.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/rover_workspace
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Note: This serves as a starting point for implementing indoor autonomous navigation using the specified sensors. Our goal is to provide a simple yet effective solution that can be extended and customized based on specific project requirements. 

## Setting Up the Sensors
### Configuring UDEV Rules
**Note:** To follow the intructions mentioned below, you need to install the [rover_install_scripts_ros2](https://github.com/RoverRobotics/rover_install_scripts_ros2) or you can do it on your own from [scratch](https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux)

Edit the ``55-roverrobotics.rules``, which can be found in the ``rover_install_scripts_ros2`` within ``udev`` folder.
You can see that ``rplidar`` has been already set up under ``# Sensor Udev Rules``. Let's setup the ``bno055``.
```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="<enter_the_vendor_id>", ATTRS{idProduct}=="<enter_the_product_id>", MODE:="0777", SYMLINK+="bno055"
```
Copy the line mentioned above under ``# Sensor Udev Rules`` and enter the vendor and product ID of your sensors using ``lsusb``. (Refer [lsusb](https://linuxhint.com/use_lsusb_command/))

```bash
cd ~/rover_install_scripts_ros2/udev
sudo cp 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Now, you should be able to see the ``bno055`` and ``rplidar`` in the list of your usb devices, using ``ls /dev``

**Note:** If you have two devices with the same vendor and product ID, you can use ``ATTR{serial}`` to differentiate between the two devices. Use ``lsusb -v`` for the same.

### Enabling and Setting Up the Ports of the Sensors
Enable the ``rplidar`` and ``bno055`` within the ``accessories.yaml`` file, which can be found in the ``roverrobotics_driver`` package within the ``config`` folder.

You can do this by setting the ``active`` parameter under ``ros__parameters`` of both the sensors as ``true``.

In the same file ``accessories.yaml`` you can update the ``serial_port`` for ``rplidar`` as ``serial_port: "/dev/rplidar"`` and similarly for ``bno055``, ``uart_port: "/dev/bno055"``.

Do not forget to perform a build of your workspace:
```bash
cd ~/rover_workspace
colcon build
```

## Robot Description Setup
Our ROS2 packages now implement URDF setups for all Rover Robots! The ``roverrobotics_description`` package implements all of the URDF configs and launches. You can view a URDF using the following:
```bash
ros2 launch roverrobotics_description display_<robot>.launch.py
```
*Valid ``<robot>`` options are: ``2wd_rover, 4wd_rover, flipper, mini, mini_2wd, miti, miti_65, max, mega``*

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
ros2 launch roverrobotics_driver navigation_launch.py map_file_name:=<path_to_map_file>
```

If you are using simulation, specify the use_sim_time parameter to be true:
```
ros2 launch roverrobotics_driver slam_launch.py use_sim_time:=true
ros2 launch roverrobotics_driver navigation_launch.py use_sim_time:=true map_file_name:=<path_to_map_file>
```

Note the parameter map_file_name. When using nav2, it is required to specify the absolute path to your map without any extension. Please use the slam toolbox plugin in rviz2 to save and serialize map files. For instance, if I saved my map as my_map, I would have the following files:

my_map.pgm
my_map.yaml
my_map.posegraph
my_map.data

To properly launch nav2, I would run:

``ros2 launch roverrobotics_driver navigation_launch.py map_file_name:=/path/to/map/my_map``

Note, I omit any extensions when specifying the map and just use the name of the map. This way slam toolbox localization loads the map using all files specified above.

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


---

## Release Notes — September 2026

This release is a reliability and driving-quality update for every CAN robot (Mini, MITI, MAX and MEGA). It makes wheel speed and odometry correct, fixes a runaway-wheel defect, makes stops smooth and battery-safe, adds a controller emergency stop, and brings the Humble and Jazzy branches to the same driver code. Every change listed here was tested on hardware before release.

### Highlights

- **Smooth, battery-safe stopping.** A new braking band removes the jolt at the end of a stop and keeps regenerative braking within what the battery accepts, including from full speed.
- **Correct speed and odometry on every robot.** Wheel speed was under-reported by 10% on the Mini and MITI and by 40% on the MAX and MEGA. It is now exact, and odometry measures true distance.
- **Emergency stop on the controller.** Circle stops the robot; Triangle resets it. Works on PS4 and PS5 controllers.
- **Retuned motor control.** New PID gains for the Mini, MITI and MAX, tuned on hardware with the corrected speed feedback.
- **One driver for Humble and Jazzy.** Both branches now carry identical driver code and configs.

### What's new

- **Braking band** (``brake_band_duty``, ``brake_band_rpm``, ``rpm_per_duty``). Bounds how hard the wheel controller may brake a rolling wheel. Enabled on the MAX 130 and MAX 150; off by default elsewhere. See *Stopping and braking*.
- **Configurable rest release** (``rest_wheel_rpm``). The speed below which a stopped wheel is released, previously fixed in code.
- **Optional release hold** (``release_hold_s``). Keeps the motors braked briefly after the wheels read zero; off by default.
- **Controller emergency stop.** ``topics.yaml`` maps Circle to ``/soft_estop/trigger`` and Triangle to ``/soft_estop/reset``. The input manager gained a button-to-``std_msgs/Bool`` topic type to support it.
- **Per-wheel joint states.** ``/joint_states`` now publishes each wheel's angle and true angular velocity.
- **Odometry reset.** Publish to ``/roverrobotics_driver/reset_odometry`` to zero the pose without restarting the driver.
- **Unit serial number.** A new ``serial_number`` parameter is published once, latched, on ``rover_<robot_type>/serial_number``.
- **Per-wheel trims** (``wheel_trim_fl`` / ``fr`` / ``rl`` / ``rr``) to balance a wheel that runs fast or slow.
- **Pose covariance parameters** (``pose_linear_covariance``, ``pose_yaw_covariance``) so sensor-fusion nodes weight wheel odometry correctly.
- **Command timeout and deceleration limit** (``cmd_vel_timeout_sec``, ``max_velocity_step``). The robot stops by itself if its velocity publisher goes quiet.
- **Controller speed overrides.** The PS5 launch accepts ``lin_increment``, ``ang_increment``, ``max_lin_speed``, ``max_ang_speed``, ``start_lin_throttle`` and ``start_ang_throttle`` so a robot can adjust its teleop feel without editing shared files.

### Bug fixes

- **Fixed a wheel that could run away.** The driver stored motor commands in a four-element array indexed by VESC IDs 1 to 4, so the rear-right wheel's command was written past the end of the array. This could corrupt neighbouring data, send a wheel an unintended command, and trigger a false "no data from robot" shutdown.
- **Fixed wheel speed and odometry scale.** Electrical-to-mechanical RPM conversion was hardcoded for one motor type. It now uses the new ``motor_pole_pairs`` parameter (15.0 on the Mini and MITI, 10.0 on the MAX and MEGA) and was verified at a 1.000 ratio against measured distance on hardware.
- **Fixed the jolt and rocking at the end of a stop.** On most stops the wheel controller briefly commanded reverse duty to a wheel that was still turning, which plugged the motor. With the braking band enabled this no longer happens: in ground testing, reverse commands to a rolling wheel dropped from 54% of stops to none.
- **Fixed battery over-voltage when stopping from high speed.** Hard regenerative braking could open the battery's protection circuit and drive the bus from 42 V to over 72 V, after which the motors cut out and the robot coasted. With the braking band, stops from full speed peaked at 42 V in testing.
- **Fixed a stop being ignored when a planner sends a near-zero command.** A command such as ``1e-9`` from a navigation stack was not recognised as a stop, which disabled the release at rest. Commands below 0.001 are now treated as stops.
- **Fixed duty wind-up.** The accumulated wheel duty is now clamped to the motor limit, so a wheel that slipped or stalled can no longer build up seconds of hidden full-power command.
- **Fixed a clean restart after an emergency stop or command timeout.** Controller state is now cleared while stopped, so the robot does not jump when it resumes.
- **Fixed the command timeout depending on the wall clock.** It now uses a monotonic clock, so a time correction after boot can neither trigger nor disable it.
- **Invalid velocity commands are rejected.** ``NaN`` or infinite values are ignored and logged instead of being sent to the motors.
- **The driver now shuts down cleanly.** Worker threads are stopped and joined, the CAN socket is closed, and CAN reads time out, so stopping or restarting the service no longer hangs.
- **Fixed an undefined result from the VESC message parser** for unrecognised frames.
- **Fixed ``gear_ratio`` of zero** causing division by zero; values of zero or less are now treated as 1.0.
- **Fixed the control mode setting being ignored.** ``control_mode`` is now honoured, and ``OPEN_LOOP``, which would command full duty on the CAN robots, is refused with a warning.
- **Fixed odometry pose lag and yaw drift.** The pose is now integrated from instantaneous wheel speed, and yaw is wrapped to [-pi, pi].

### Improvements

- **Stable CAN interface naming.** All CAN configs now use ``rovercan``, a fixed name given to the USB-CAN adapter by a udev rule, so the driver can no longer bind to an unused onboard CAN controller after a reboot.
- **PS5 is the default controller** in every teleop launch. PS4 remains fully supported.
- **Deceleration tuned on hardware.** ``max_velocity_step`` is 0.75 on every robot; the earlier 0.05 made the robot coast after the stick was released.
- **Clearer startup logging.** The driver logs its gear ratio, pole pairs, control mode, braking settings and serial number at startup, and warns about invalid settings.

### Changes to be aware of

- **Retune custom gains.** Correcting the speed feedback changed the effective loop gain by about +11% on the Mini and MITI and +67% on the MAX and MEGA. Gains tuned against the old feedback should be retuned; the shipped gains already are.
- **MAX 6.5 inch and 10 inch variants are no longer supported.** Their configs and URDFs were removed. The MAX is supported with 13 inch and 15 inch wheels.
- **``device_port`` is now ``rovercan``.** Manual installs must install the udev rule described under *Connection*.
- **Parameters must be written as decimals** (``15.0``, not ``15``) or the driver will not start.
- **Stopping from 160 to 215 rpm takes 0.1 to 0.25 s longer on the MAX** with the braking band enabled, and stops from full speed take about 2.1 to 2.9 s. This is the cost of keeping regenerative braking within what the battery accepts.

### Known issues

- An emergency stop at full speed brakes as hard as the motors allow and briefly raised the bus to about 55 V in testing. It is safe to use, but it should not be the routine way to stop at top speed.
- ``/robot_info`` is still only published on request, and its request subscriber is bound to the estop reset topic.
- ``battery_status.current`` is not a battery current and is currently mis-decoded; see the ``battery_status`` section. The robots have no pack current sensor, so charging cannot be detected.
- On some JetPack 5 systems, Fast DDS can stop delivering messages between processes shortly after start. Use Cyclone DDS as described under *Troubleshooting*.
- The braking band's default values were measured on a MAX 130. Confirm ``rpm_per_duty`` on the first MAX 150 before relying on it for hard stops.

### Development timeline

A dated record of the work in this release, for reference.

| Date | Work | Verified on |
| --- | --- | --- |
| 2026-09-16 | Fixed the runaway-wheel defect (out-of-range motor command array). Added clean driver shutdown: thread stop and join, CAN read timeout, virtual destructors, and a safe default in the VESC message parser. | Drive-tested on an AGX Thor test rover |
| 2026-09-17 | Corrected wheel speed and odometry scale with the new ``motor_pole_pairs`` parameter. Standardised CAN naming on ``rovercan``. Removed the MAX 6.5 and 10 inch variants. Made PS5 the default controller. Set ``max_velocity_step`` to 0.75 on every robot. Added the command timeout, per-wheel trims, serial number and pose covariance parameters. | Odometry ratio 1.000 on a MITI and a MEGA; full change set validated on a freshly installed MITI |
| 2026-09-21 | Brought up a Mini on an Orin Nano Super with ROS 2 Jazzy from this release; CAN communication verified. | Mini |
| 2026-09-22 | Tuned the Mini's PID gains (P 0.0008, D 0.00006) and verified the PS5 controller over Bluetooth. Tuned the MAX 130 with a 50 to 70 lb payload across nine recorded runs (P 0.0012, D 0.00006). Identified that the remaining stop jolt was not caused by the gains. | Mini; MAX 130 |
| 2026-09-23 | Traced the stop jolt to reverse duty sent to still-rolling wheels, and found that hard stops from high speed were tripping the battery protection. Built a simulator from recorded CAN data to evaluate fixes, rejected a first design that failed at speed, and developed the braking band. Added the controller emergency stop. | Stand tests and ground tests on the MAX 130 with payload |
| 2026-09-24 | Traced intermittent loss of controller input to the Fast DDS shared-memory transport and moved the test robot to Cyclone DDS. Tested the emergency stop from speed. Finalised and cleaned up the driver code, applied the MAX 130 settings to the MAX 150, merged the release into the Humble and Jazzy branches, and updated this documentation. | MAX 130 |
