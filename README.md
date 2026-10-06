
# ROS 2 Driver for Rover Robots

  

## About:

  

- This package is being created to add necessary features and improvements for our robots, specifically for ros2. Our ros2 package has been lacking in regards to out-of-the-box support for items such as URDF, Simulation, Slam, and Navigation. This package aims to bring our ros2 up to speed with all of these features.

  

- This package is exclusively built for ROS2. The ``jazzy`` branch is tested on Ubuntu 24.04 with ROS 2 Jazzy (JetPack 7 on NVIDIA Jetson); the ``humble`` branch is tested on Ubuntu 22.04 with ROS 2 Humble (JetPack 6 on NVIDIA Jetson). Use the branch that matches your Jetson's JetPack version.

  

- This is built on top of the old roverrobotics_ros2 driver. It is designed specifically to fix many reoccuring bugs that we faced with the old driver and implement new features.

  

- Stable releases are published on a branch per ROS 2 distribution. Use the branch matching your distribution (``humble`` or ``jazzy``) rather than the development branch.

  
  

## New here? Start with this

This section is for anyone who has never used ROS. It takes you from a fresh computer to a driving robot, and shows where everything lives. You do not need to read the rest of this page to get started.

### What you need

- A Rover robot (Mini, MITI, MAX, MEGA, Pro or Zero).
- One computer that runs ROS and drives the robot, connected to it (for example an NVIDIA Jetson mounted on the robot). It must run **Ubuntu 24.04** for the ``jazzy`` branch or **Ubuntu 22.04** for the ``humble`` branch (on a Jetson: JetPack 7 or JetPack 6). The install script picks the right branch for you. The robot does not include a computer.
- An internet connection on that computer, and a keyboard and screen or an SSH connection to it.
- A PS5 or PS4 controller to drive with.

### ROS words in plain English

You will see these words below and in the rest of this page:

| Word | What it means here |
| --- | --- |
| **ROS 2** | The software framework the robot runs on. It is installed for you. |
| **Workspace** | The folder that holds the robot's software: ``~/rover_workspace``. |
| **Package** | One folder of related software inside the workspace, for example the driver. |
| **Config file** | A text file (``.yaml``) with settings you can edit, such as top speed or which sensors are on. |
| **Build** | Turning the source files into the version the robot runs. Needed after you edit a config file. |
| **Launch file** | A file that starts several programs together, for example the driver plus the controller. |
| **Topic** | A named stream of data, such as ``/imu/data`` for the IMU or ``/scan`` for the LiDAR. |
| **Service** | ``roverrobotics.service``: starts the robot software automatically every time the computer boots. |

### Step 1: Install everything

The install script does all the work. On the computer connected to the robot, open a terminal and run:

```bash
git clone https://github.com/RoverRobotics/rover_install_scripts_ros2
cd rover_install_scripts_ros2
./setup_rover.sh
```

It asks a few questions. Good answers for a first install:

- **Robot:** pick your robot. A MAX also asks for its wheel size (13 or 15 inch).
- **Controller:** PS5 or PS4, whichever you have.
- **Start at boot (service):** yes. The robot is then ready to drive every time it powers on.
- **IMU (BNO055)** and **LiDAR (RPLIDAR S2):** say yes if your robot has them. This installs their software; you switch them on in Step 5.
- **RealSense camera:** yes if your robot has one.

The install takes from about 15 minutes to an hour, depending on the computer and the options. Progress is shown on screen and saved to ``~/rover_setup.log``. Reboot when it finishes.

### Step 2: Drive

1. **Put the robot on a stand with the wheels off the ground for your first test.**
2. Pair the controller once: on the computer run ``bluetoothctl``, then ``pairable on`` and ``scan on``, hold the controller's pairing buttons (PS5: Create + PS; PS4: Share + PS) until it flashes, then ``pair <address>``, ``trust <address>``, ``connect <address>`` and ``exit``.
3. Drive with the left stick (forward and back) and the right stick (turning).
4. **Cross (✕) is the emergency stop. Circle (○) releases it.** Try both before driving on the ground.

All buttons are listed under *Controller buttons* below.

### Step 3: Everyday commands

Open a terminal on the computer and use these:

| To do this | Run |
| --- | --- |
| See whether the robot software is running | ``systemctl status roverrobotics`` |
| Restart it (after a settings change, or if it misbehaves) | ``sudo systemctl restart roverrobotics`` |
| Stop it | ``sudo systemctl stop roverrobotics`` |
| Start it again | ``sudo systemctl start roverrobotics`` |
| Watch its messages live (Ctrl+C to quit) | ``journalctl -u roverrobotics -f`` |
| List all data streams (topics) | ``ros2 topic list`` |
| Check that a stream is updating, and how fast | ``ros2 topic hz /odometry/wheels`` |
| Print the data of a stream | ``ros2 topic echo /odometry/wheels`` (Tab completes names) |

If you did not install the service, start the robot by hand instead, and keep that terminal open while you drive:

```bash
ros2 launch roverrobotics_driver <robot>_teleop.launch.py
```

Replace ``<robot>`` with ``mini``, ``miti``, ``max``, ``mega``, ``pro``, ``zero``, ``mini_2wd`` or ``miti_65``.

### Step 4: Change a setting

Every setting lives in a config file in the driver package. The general recipe is always the same:

1. **Edit** the file, for example with ``nano``:
   ```bash
   nano ~/rover_workspace/src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml
   ```
   Save with Ctrl+O and Enter, quit with Ctrl+X. Keep the indentation (spaces at the start of lines) exactly as it is.
2. **Build**, so the robot uses the edited file:
   ```bash
   cd ~/rover_workspace
   colcon build
   ```
3. **Restart** the robot software:
   ```bash
   sudo systemctl restart roverrobotics
   ```

Edits in ``~/rover_workspace/src`` do nothing until you build. If a change seems to have no effect, you probably skipped the build.

### Step 5: Turn on your sensors

The IMU, LiDARs and camera are all switched on in one file, ``accessories.yaml``. Each one has its own short guide under *Adding Sensors* below, written for beginners.

### Where things are

Everything is inside ``~/rover_workspace/src/roverrobotics_ros2``:

| Folder or file | What it holds | Do you edit it? |
| --- | --- | --- |
| ``roverrobotics_driver/config/<robot>_config.yaml`` | Your robot's settings: speed limits, wheel size, motor control | Sometimes |
| ``roverrobotics_driver/config/accessories.yaml`` | Which sensors are on, and their settings | Yes, to add sensors |
| ``roverrobotics_driver/config/topics.yaml`` | What each controller button does | Rarely |
| ``roverrobotics_driver/launch/`` | Launch files that start the robot (``max.launch.py``, ``max_teleop.launch.py`` and so on) | Rarely |
| ``roverrobotics_description/urdf/`` | The 3D model of each robot and where each sensor is mounted | Only if you move a sensor |
| ``roverrobotics_driver/src`` and ``library/`` | The driver's source code | No |
| ``roverrobotics_gazebo/`` | Simulation | Only for simulation |

The MAX has one config per wheel size: ``max_130_config.yaml`` for 13 inch wheels and ``max_150_config.yaml`` for 15 inch. The install script points the MAX launch files at the right one.

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
| **Cross (✕)** | **Software emergency stop.** Latches the robot stopped until reset. |
| **Circle (○)** | **Reset the emergency stop.** The robot resumes on the next stick input. |

The estop buttons work on both the PS4 and the PS5 controller. They are defined in ``config/topics.yaml`` by button name (``A`` for Cross, ``B`` for Circle), and every controller config maps those names to the correct physical buttons for its kernel driver, so no per-controller change is needed. A single press sends a single message; holding the button does not repeat it.


#### When the controller goes out of range

If a controller connected to the robot goes out of range, its link can stall without disconnecting, and the joystick driver keeps repeating the last stick position ([ros-drivers/joystick_drivers#92](https://github.com/ros-drivers/joystick_drivers/issues/92)), so the robot would keep driving. The input manager watches the controller's own report stream instead: a connected PS4 or PS5 controller sends several hundred reports a second even with the sticks still. If none arrive for 0.5 s, it sends one stop command and ignores the sticks until the controller is back **and** the sticks are centred. It needs read access to the controller's raw device, which the install scripts' udev rules give; without it, it logs a warning and the controller works as before. Commands from anything else, such as navigation or remote teleoperation, are not affected.

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
| ``motor_pole_pairs`` | Pole pairs in the drive motor, i.e. half the pole count. A VESC reports electrical RPM, and mechanical RPM is electrical RPM divided by this. **Mini and MITI are 30-pole, so 15.0; MAX and MEGA are 20-pole, so 10.0.** Getting it wrong scales every wheel speed and all of wheel odometry by the same factor. |
| ``gear_ratio`` | Motor revolutions per wheel revolution on geared drivetrains. Applied to the RPM feedback to convert motor RPM to wheel RPM. The MEGA and MAX are geared; the Mini and MITI are direct drive and must stay at ``1.0``, which makes the conversion a no-op. Values of zero or less are rejected and treated as ``1.0``. |

The MAX is supported with 13 inch (``max_130_config.yaml``, nominal radius 0.1651, calibrated effective radius 0.155) and 15 inch (``max_150_config.yaml``, radius 0.1905) wheels. The 6.5 inch and 10 inch variants are no longer supported and their configs and URDFs have been removed. Selecting the wrong config silently scales odometry and commanded velocity, so confirm the radius matches the wheels actually fitted.

**Effective track width on skid-steer robots.** The tyres slide sideways in a turn, so the robot rotates less than its wheel speeds imply. Setting ``wheel_base`` to the *effective* track width instead of the measured distance between wheel centers corrects both sides at once: a commanded turn rate produces that turn rate, and wheel odometry reports the rotation that actually happens. To measure it, command a pivot (for example 1.0 rad/s), record the true wheel speed and the IMU yaw rate once settled, and compute ``2 × wheel surface speed ÷ IMU yaw rate``. On a MITI (measured wheel base 0.387 m) this gave **0.60 m**: pivots went from 66% to 91% of the commanded rate and odometry yaw matched the IMU within 1% in arcs. On a MAX 130 (measured wheel base 0.4953 m) it gave **0.90 m**: pivots went from 53% to 99% and arcs from 49% to 98% of the commanded rate. The value depends on tyres and floor, so navigation should still fuse IMU yaw.

**Effective wheel radius.** A loaded tyre rolls on a slightly smaller radius than its nominal size. Drive a measured straight line, compare the tape distance with the wheel revolutions counted by the VESC tachometer, and set ``wheel_radius`` to the result. On the MAX 130 this gave 0.155 instead of the nominal 0.1651, and moved odometry from +5.8% to within 1% of the tape.

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
| ``max_linear_acceleration`` | Gentle start, CAN robots, opt-in (``0`` = off, the default). While the commanded forward speed is rising, the command given to the wheel controller climbs at this rate in m/s^2 instead of the built-in 5 m/s^2. Slowing down and stopping are not affected. The command is ramped, not the measured speed, so a speed dip caused by turning is corrected at the normal rate. **1.5 on the MAX 130.** |
| ``max_angular_acceleration`` | The same for turning, in rad/s^2 (``0`` = off, built-in 30 rad/s^2). Applies while a turn builds up; easing off a turn is immediate, and reversing the turn direction goes through zero first. **4.0 on the MAX 130**, where it lowered the peak motor current at the start of a pivot from standstill from 37 A to 21 A (median). |

The timeout is a safety stop for a lost or stalled publisher. Any node commanding the robot must publish continuously, not once per change of speed.

The timeout is measured on a monotonic clock, so a wall-clock jump (for example NTP correcting the time shortly after boot) can neither trigger it falsely nor disable it. Velocity commands containing ``NaN`` or infinity are rejected and logged, and do not refresh the timeout. A command smaller than 0.001 in both ``linear.x`` and ``angular.z`` is treated as a stop, so a planner that publishes a tiny residual such as ``1e-9`` still brings the robot fully to rest.

The timeout is logged once, as ``cmd_vel timeout ... HALT.``, when commands stop arriving after the robot has been driven; it is not repeated while the robot sits idle.

### Stopping and braking

These parameters shape how a CAN robot comes to rest when the commanded speed drops. The braking band is off unless a config enables it, so a robot without these keys behaves exactly as before.

| Parameter | Description |
| --- | --- |
| ``rest_wheel_rpm`` | Below this wheel speed, a wheel on a commanded stop is released: its duty is zeroed and its PID reset, so leftover duty cannot rock it back and forth. Default ``8.0``. Accepted range (0, 30]. |
| ``brake_band_duty`` | Enables the braking band (``0.0`` = off). While a wheel is rolling faster than its target, its duty is not allowed to fall more than this far below the duty that would hold its current speed. This bounds the braking current, and with it the energy pushed back into the battery. Valid 0.0 to 0.5. |
| ``brake_band_rpm`` | Above this wheel speed the band narrows in proportion to 1/rpm, so braking current stays roughly constant as speed rises instead of growing with it. |
| ``rpm_per_duty`` | Wheel rpm produced by one unit of duty with no load, i.e. the motor's back-EMF constant expressed at the wheel. The band is computed from it. With the band enabled, values outside 300 to 340 are rejected as a likely typo and the band is switched off. |
| ``release_hold_s`` | Optional. Keeps the motors actively braked for this many seconds after the wheels read zero before releasing them, which can help hold the robot on a slope. ``0.0`` (default) releases immediately. Accepted range 0 to 1. |
| ``brake_momentum_carry`` | Opt-in (default ``false``). The band hands a wheel back to the PID when its speed rises during braking, which is how a downhill roll is detected. A robot released in the middle of a short push keeps speeding up for about 0.1 to 0.2 s from momentum, which that rule mistook for a hill, so some stops were firm and others soft. With this on, a rise within the first 200 ms of a stop is followed rather than treated as a hill; a real downhill roll is still handed to the PID, at most 200 ms later. **On for the MAX 130.** |

**Why the braking band exists.** The wheel controller accumulates duty and lowers it gradually on a stop. Without the band it overshoots through zero and briefly commands the opposite direction while the wheel is still turning, which on a VESC means plugging the motor: the robot stops with a sharp kick and can rock. At high speed the same controller can brake hard enough to push the battery's regenerative current past what its protection circuit accepts, which opens the charge path and lets the bus voltage climb far above the pack voltage. The band removes both: duty lands on zero at low speed and the motor's own short-circuit brake finishes the stop, and braking current is capped at speed.

**Shipped values.** Only the MAX configs enable the band:

```yaml
    rest_wheel_rpm: 8.0
    brake_band_duty: 0.10         # MAX 150; 0.20 on the MAX 130
    brake_band_rpm: 160.0
    brake_momentum_carry: false   # true on the MAX 130
    rpm_per_duty: 325.0
    release_hold_s: 0.0
```

**Choosing ``brake_band_duty``.** The band sets how firmly the robot stops. On an unloaded MAX 130, 0.10 gave about 1.1 m/s^2 (1.8 s from 2.2 m/s) and 0.20 gave about 2.4 m/s^2 (0.9 s from 2.2 m/s). With 0.20 and ``brake_momentum_carry`` on, all 35 pad stops in a recorded drive stayed on the band with a peak motor current of 15 to 31 A, and the bus voltage stayed within 1.7 V of the pack voltage.

**Calibrating ``rpm_per_duty``.** Put the robot on a stand with the wheels clear of the ground, drive it at three or four steady speeds (for example 1, 2, 3 and 3.8 m/s) and read the wheel rpm and duty from the VESC status frames. Divide rpm by duty at each speed and use the value measured at the higher speeds. On the MAX 130 this measured 322 to 329, so 325 is used. Stay within 320 to 335 unless you have measured otherwise: higher values brake harder and can trip the battery protection, lower values brake more softly and can let the robot roll further downhill.

The band applies in ``INDEPENDENT_WHEEL`` mode only. It does not act during an emergency stop, which always brakes as hard as the motors allow.

### Speed feedback at low speed

On CAN robots the driver takes each wheel's speed from the VESC status message (electrical RPM). With hall-sensored motors, the VESC reports roughly **half the real speed below its *Hall Interpolation ERPM* setting** (VESC Tool, Motor Settings → FOC → Hall Sensors; default 500) and the correct speed above it. Everything built on that value is then wrong at low speed: the wheel controller holds the under-reading at target, so the robot drives too fast, and odometry under-reports distance by the same factor.

Measured on a MITI (direct-drive hub motors, 15 pole pairs), true speed from the VESC tachometer ÷ reported speed:

| Command | True eRPM | Interp. 500 | Interp. 150 | Interp. 50 |
| --- | --- | --- | --- | --- |
| 0.05 m/s | 57 | – | 2.1 | 1.2 |
| 0.1 m/s | 113 | 2.2–2.6 | 2.1 | 1.03–1.09 |
| 0.2 m/s | 228 | 1.5–2.3 | 1.00 | 1.00 |
| 0.4 m/s | 452 | 1.01–1.11 | 0.99 | 1.00 |
| 0.8 m/s | 905 | 0.99 | 0.99 | 0.99 |

**Fix: set *Hall Interpolation ERPM* to 50 on every VESC** and write the configuration. On the MITI this brought odometry within 0.5% of a tape measurement at 0.2 m/s (it was 32% short before). Below about 100 eRPM (≈0.07 m/s on a MITI) the reported speed is still unreliable because too few hall edges arrive. The setting lives in the VESCs, not in this repository, so it must be applied to each robot. Direct-drive robots are the most affected; geared robots spin their motors faster at the same ground speed, but a MAX with hall sensors at the default setting is still affected below about 0.18 m/s.

``use_tachometer_speed`` (default ``false``) is a fallback that computes each wheel's speed from the VESC tachometer (CAN status 5) instead. It is correct at every speed but updates more slowly at low speed, so it is best left off once the VESC setting above is applied. It requires status 5 to be enabled on every VESC; a wheel whose tachometer goes quiet falls back to the status value.

**Where to see the tachometer.** No topic carries the tachometer on its own. With ``use_tachometer_speed: true`` the tachometer speed *replaces* the wheel speed everywhere the driver publishes it: ``/joint_states`` ``velocity``, ``/robot_status`` indices 1, 6, 11 and 16, and the wheel odometry. With it ``false`` (the default) the driver still decodes the tachometer but publishes nothing from it. To compare the two sources directly, read the CAN bus:

```bash
candump -ta rovercan,1B00:1FF00   # status 5 frames, ID 0x1B0N for VESC N
```

Bytes 0-3 are the tachometer, a signed 32-bit count that grows by 6 per electrical revolution (on a MITI, 6 × 15 = 90 counts per wheel revolution); bytes 4-5 are the bus voltage × 10. Wheel rpm = change in count ÷ 6 ÷ seconds × 60 ÷ ``motor_pole_pairs`` ÷ ``gear_ratio``. VESC Tool shows the same counter as *Tachometer* in its realtime data. Status 5 must be enabled on each VESC (VESC Tool, App Settings → General → CAN Status Message Rate).

### Feedforward and launch control

Feedforward gives each wheel the duty its target speed needs straight away, from a calibrated model of the motor, so the PID only has to correct the small remainder. The result is faster response without overshoot and a quieter drive at low gains. It applies to ``INDEPENDENT_WHEEL`` mode, is opt-in per robot, and is enabled on the MITI and the MAX 130. Every parameter defaults to off; with ``ff_rpm_per_duty`` at 0 the controller behaves exactly as without it.

| Parameter | Description |
| --- | --- |
| ``ff_rpm_per_duty`` | Enables feedforward. Each wheel is given ``ff_static_duty + abs(target rpm) / ff_rpm_per_duty`` immediately, and the PID only corrects the remainder. Measured under load on the ground, not on a stand. |
| ``ff_static_duty`` | Duty needed just to keep the wheels rolling (friction). |
| ``ff_turn_duty`` | Extra duty that helps a pivot break the tyres free. It acts only on the turning part of a command, so it adds no forward push in an arc, and fades out as each wheel reaches its target. |
| ``ff_calibration_voltage`` | Battery voltage at which the feedforward was calibrated (0 = off). The feedforward is scaled by ``calibration voltage ÷ battery voltage`` (limited to 0.7–1.4), so the same duty-per-speed holds as the battery drains or on a fuller pack. Uses the voltage the VESCs report. |
| ``low_speed_trust_rpm`` | Below this wheel speed the speed feedback is treated as unreliable: after a command change the PID stays out for the full launch window and then runs at 25% strength. Around 100 eRPM expressed in wheel rpm (7.0 on a MITI). |
| ``wheel_speed_filter`` | Smoothing of the speed the PID sees (0 = off, 0.5 = moderate). Does not affect odometry. |
| ``ff_correction_decay`` | Per-cycle decay applied to the PID's correction on top of the feedforward (``0`` = the standard output decay of 0.989; ``1.0`` = no decay; otherwise 0.9 to 1.0). With the standard decay the correction leaks away, which leaves a steady speed error; on the MAX 130 the inner wheels of an arc ran 11 to 17% fast. ``1.0`` removed that error and halved the current at the start of an arc (34.7 A to 18.9 A). |
| ``ff_correction_release`` | Opt-in (default ``false``). The launch hold keeps the PID out after a command change, which also froze a correction that no longer fits. With this on, a stale correction is cleared at once: when a wheel still rolling one way must reverse (driving forward, then pivoting), and when a correction points away from the wheel's new target (after a turn ends). On the MAX 130 it shortened the forward drift before a pivot from 0.16 m to 0.08 m, and the dip in forward speed after a turn while weaving from 20% (up to 49%) to 4% (up to 8%). |

With feedforward on, the controller also: ramps the feedforward toward the command at the acceleration limits; keeps the PID out after each command change until that wheel reaches 85% of its target (at most 0.6 s), which removes the launch overshoot; never drives a rolling wheel backwards on a stop; and only cuts small duties to zero (a VESC brake) for wheels meant to be stopped, which removes the stop-go jerk when crawling. Emergency stop, stale feedback and the command timeout bypass all of it. Odometry and ``/joint_states`` are never affected by these parameters.

**Calibration.** On the ground, drive steady speeds in both directions (for example ±0.1, 0.2, 0.4 and 0.6 m/s), record the settled duty and true wheel rpm from the VESC status frames, and fit ``duty = static + rpm / rpm_per_duty``. The values depend on the robot model, its tyres and the floor; set ``ff_calibration_voltage`` to the battery voltage during calibration. After calibrating, retune the gains lower (see *Motor control gains*). The complete MITI settings, calibrated at 39.4 V:

```yaml
    wheel_base: 0.60              # effective track width, see Kinematics
    motor_control_p_gain: 0.0002
    motor_control_i_gain: 0.00002
    motor_control_d_gain: 0.00002
    ff_rpm_per_duty: 877.0
    ff_static_duty: 0.0072
    ff_turn_duty: 0.06
    ff_calibration_voltage: 39.4
    wheel_speed_filter: 0.5
    low_speed_trust_rpm: 7.0
    use_tachometer_speed: false
```

The MAX 130 settings, calibrated at 40.3 V without payload (*Hall Interpolation ERPM* 50 on every VESC):

```yaml
    wheel_radius: 0.155           # effective radius, see Kinematics
    wheel_base: 0.90              # effective track width, see Kinematics
    motor_control_p_gain: 0.0005
    motor_control_i_gain: 0.0
    motor_control_d_gain: 0.000025
    ff_rpm_per_duty: 322.0
    ff_static_duty: 0.021
    ff_turn_duty: 0.0
    ff_calibration_voltage: 40.3
    ff_correction_decay: 1.0
    ff_correction_release: true
    low_speed_trust_rpm: 2.0
```

On the ground this drove 3.015 m on the tape for 3.0 m commanded, with odometry within 0.5%, pivots at 102% and arcs at 103% of the commanded turn rate.

### Diagnostics

| Parameter | Description |
| --- | --- |
| ``odometry_frequency`` | Publish rate for the wheel odometry topic, in Hz. |
| ``linear_covariance`` / ``yaw_covariance`` | Uncertainty published on the odometry **twist**, i.e. the measured velocity. |
| ``pose_linear_covariance`` / ``pose_yaw_covariance`` | Uncertainty published on the odometry **pose**. The pose is dead reckoned from wheel rotation, so it drifts and these must not be zero. A fusion node reads an all-zero covariance as "no uncertainty" and will trust wheel odometry over every other sensor. |
| ``robot_status_frequency`` | Publish rate for the robot status topic, in Hz. |

### Battery

| Parameter | Description |
| --- | --- |
| ``battery_cells`` | Cells in series in the battery pack: 10 on the Mini, MITI, MAX and MEGA (36 V nominal, 42 V full), 4 on the Zero. |
| ``battery_max_cell_voltage`` | Cell voltage reported as 100%. Default ``4.2``. |
| ``battery_min_cell_voltage`` | Cell voltage reported as 0%. Default ``3.4``: 34 V on the standard 10-cell (36 V nominal) pack, which leaves a small reserve above the battery protection cut-off. |
| ``battery_voltage_multiplier`` | Correction for the voltage the motor controller reports, measured against a meter. ``1.025`` on the MITI, ``1.0`` elsewhere. |

The percentage these produce is described under ``battery_status``.

### Motor control gains

| Parameter | Description |
| --- | --- |
| ``motor_control_p_gain`` | On the CAN robots the controller output is **added** to the previous duty every cycle, so this gain behaves as the integral term: it sets how quickly duty ramps toward the target and how small the steady-state speed error is. |
| ``motor_control_i_gain`` | Because the output is accumulated, this acts as a double integrator. Keep it at zero unless feedforward is enabled; with feedforward the MITI uses a small value to remove the last steady-state error. Larger values cause overshoot. |
| ``motor_control_d_gain`` | On the CAN robots this behaves as the proportional term, and provides the damping. Too low and the wheels overshoot and ring after a speed change; too high and turning in place goes unstable. |

Retune the gains if you change ``motor_pole_pairs``, ``gear_ratio``, wheel size, motors or the VESC *Hall Interpolation ERPM*. The controller's feedback is the measured wheel speed, so anything that changes that number changes the effective loop gain.

Shipped gains, tuned on hardware:

| Robot | ``p_gain`` | ``i_gain`` | ``d_gain`` | Notes |
| --- | --- | --- | --- | --- |
| Mini | 0.0008 | 0.0 | 0.00006 | Tuned on a Mini, Orin Nano, ROS 2 Jazzy |
| MITI | 0.0002 | 0.00002 | 0.00002 | With feedforward. Requires *Hall Interpolation ERPM* 50 on every VESC; see *Speed feedback at low speed* |
| MAX 130 | 0.0005 | 0.0 | 0.000025 | With feedforward, tuned without payload. Without feedforward use 0.0012 / 0.0 / 0.00006 (tuned with a 50 to 70 lb payload) |
| MAX 150 | 0.0012 | 0.0 | 0.00006 | PID only, no feedforward; same drivetrain as the MAX 130, verify on the first unit |
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
| ``trim_topic`` | Rover Pro only: runtime steering trim, ``/trim_event``. On the Mini, MITI, MAX and MEGA it is ignored (the driver logs once); use ``wheel_trim_fl`` / ``fr`` / ``rl`` / ``rr`` instead. |

### Emergency stop

| Parameter | Description |
| --- | --- |
| ``estop_trigger_topic`` | A ``std_msgs/Bool`` of ``true`` here latches the robot stopped. Default ``/soft_estop/trigger``. |
| ``estop_reset_topic`` | A ``std_msgs/Bool`` of ``true`` here clears it. Default ``/soft_estop/reset``. |
| ``estop_status_topic`` | Where the current estop state is published (see below). Default ``/soft_estop/status``. |
| ``estop_state`` | State at startup. ``true`` makes the robot boot already stopped, so it cannot move until the estop is reset. ``false`` on every shipped config. |

Every shipped config lists these keys explicitly with their values, so each robot's estop setup is visible in its config file.

**How the emergency stop behaves.** When the trigger arrives, the next control cycle (30 ms) sends a duty of zero to every VESC, which short-circuits the motors and brakes as hard as they allow. Stick and ``cmd_vel`` input is ignored while the stop is latched. The wheel controller's accumulated duty, PIDs and braking state are cleared, so when the stop is reset the robot resumes from rest with no jump; it moves again only when a new, non-zero command arrives.

From the controller, press **Cross (✕)** to stop and **Circle (○)** to reset. From a terminal:

```bash
ros2 topic pub --once /soft_estop/trigger std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /soft_estop/reset   std_msgs/msg/Bool "{data: true}"
```

**Checking the estop state.** The driver publishes the current state as a latched ``std_msgs/Bool`` on ``/soft_estop/status``: ``true`` while the estop is engaged, ``false`` when released. It is sent at startup and on every change, and a subscriber that joins later receives the current value immediately.

```bash
ros2 topic echo --once /soft_estop/status     # current state
ros2 topic echo /soft_estop/status            # follow changes live
```

The driver log records the same information:

```bash
journalctl -u roverrobotics -b --no-pager --grep "Estop state is|Software Estop" | tail -1   # current state
journalctl -u roverrobotics -f --grep "Estop state is|Software Estop"                        # live
```

``Estop state is currently active/inactive`` is the state the driver started in; ``Software Estop activated/deactivated`` marks each change. Neither the topic nor the log sees a physical e-stop or power switch.

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

To change the teleop feel for one robot type without editing those shared files, add a ``joy_manager`` block to that robot's config; its ``*_teleop.launch.py`` passes it to the controller launch. The Rover Pro and the MAX use this. The MAX 130 block starts at 1.25 m/s and 1.25 rad/s at full stick, adds 0.3125 per D-pad press and stops at 1.875 on both axes:

```yaml
joy_manager:
  ros__parameters:
    start_lin_throttle: 1.0     # x the stick scale 1.25 = 1.25 m/s
    lin_increment: 0.25
    max_lin_speed: 1.875
    start_ang_throttle: 0.5     # x the stick scale 2.5 = 1.25 rad/s
    ang_increment: 0.125
    max_ang_speed: 1.875
```

``max_lin_speed`` and ``max_ang_speed`` cap the published command, not the D-pad multiplier: pressing past the cap still raises the multiplier, so partial stick moves faster until the D-pad is pressed back down. ``max_teleop.launch.py`` reads the same config file as ``max.launch.py``; if you switch ``max.launch.py`` to ``max_130_config.yaml``, switch ``max_teleop.launch.py`` too.

### `rover_<robot_type>/battery_status` (`sensor_msgs/BatteryState`)

| field | unit | notes |
| --- | --- | --- |
| ``voltage`` | volts | bus voltage measured at the motor controller |
| ``current`` | amps | input current of one motor controller, **not the battery current**; see the note below |
| ``percentage`` | **percent, 0 to 100** | estimated from voltage; see the notes below |
| ``present`` | bool | true whenever the driver has a live connection to the robot |
| ``power_supply_status`` | enum | UNKNOWN on the CAN robots; see the note below |

``header.stamp`` is set from the node clock on every publish.

**``percentage`` is deliberately 0 to 100, not the 0 to 1 that the message definition specifies.** Rover Robotics publishes the state of charge directly because it is what customers already read and it is easier to interpret in a terminal. A generic battery widget that assumes the ROS convention will therefore read it 100 times too high. Divide by 100 if you are feeding a tool that expects the standard range.

**``current`` does not measure the battery.** The robots have no battery current sensor. The value is the input current reported by one motor controller (the VESC with CAN ID 1, the only one that sends its input-current status), so it covers one of four motors and excludes the other three, the computer and accessories. It is positive while that motor drives and negative while it regenerates during braking. Use it as a rough indication of that motor's load, not of the battery.

**``percentage`` is an estimate from voltage,** mapped linearly from ``battery_min_cell_voltage`` to ``battery_max_cell_voltage`` times ``battery_cells`` after applying ``battery_voltage_multiplier`` (see *Battery*); on a 10-cell pack that is 34 V (0%) to 42 V (100%). There is no load compensation: it drops while the motors draw current and rises again at rest, so read it with the robot idle.

**``power_supply_status``** is UNKNOWN on the CAN robots (Mini, MITI, MAX and MEGA). Detecting charging needs a current sensor on the battery, which these robots do not have: a charger connected to the battery never passes through a motor controller, and the only negative current a motor controller sees is its own motor regenerating. On the Rover Pro, whose battery board reports the pack current, it is FULL at 100%, CHARGING when ``current`` is below -0.1 A, DISCHARGING above 0.1 A and NOT_CHARGING otherwise.

Fields the VESCs do not report are left at their defaults: ``temperature``, ``charge``, ``capacity``, ``design_capacity``, ``power_supply_health``, ``power_supply_technology``, ``location``, ``cell_voltage`` and ``cell_temperature``.

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

### `/soft_estop/status` (`std_msgs/Bool`)

``true`` while the software emergency stop is engaged, ``false`` when it is released. Latched (transient-local), published at startup and on every change. See *Emergency stop*.

### `/robot_info` (`std_msgs/Float32MultiArray`)

Five values: robot GUID, firmware version, speed limit, fan speed and fault flag.

Two caveats. It is **not published periodically**: it publishes only when a ``std_msgs/Bool`` of ``true`` arrives on ``robot_info_request_topic`` (default ``/robot_info/request``). And because the message is ``Float32MultiArray``, any value above 16,777,216 loses precision silently, so it is not a suitable place for a serial number. Use ``serial_number`` above instead.

### `/robot_status` (`std_msgs/Float32MultiArray`)

A flat, unlabelled array. Indices 0-19 are five values per motor in the order id, rpm, current, temperature, MOSFET temperature, for motors 1 to 4. So per-wheel RPM sits at indices 1, 6, 11 and 16. RPM here is **wheel** RPM, after the pole-pair and gear-ratio conversion. Prefer ``/joint_states`` for per-wheel work.


### Timestamps

Every message type that has a header carries a stamp set from the node clock: ``/odometry/wheels``, ``/joint_states``, ``rover_<type>/battery_status`` and the ``odom`` transform.

``/robot_status``, ``/robot_info`` and ``rover_<type>/serial_number`` use ``std_msgs`` types, which have **no header field at all**, so they cannot carry a timestamp. A timestamp also cannot be smuggled into the ``Float32MultiArray`` payload, because a ROS epoch time needs 31 bits and ``float32`` holds only 24 bits of mantissa, so the value would be silently rounded. Giving those topics a stamp would mean changing their message types.

Per-wheel RPM and battery state are both available on stamped topics already: use ``/joint_states`` and ``rover_<type>/battery_status`` when you need to correlate readings in time.

## Troubleshooting

**The robot stops responding a few seconds after the driver starts, or never responds after boot.** If ``/joy`` and ``/cmd_vel`` stop reaching the driver roughly 20 to 30 seconds after every start, while the controller stays connected, the ROS 2 middleware has stopped delivering messages between processes on the robot. On an Orin Nano running JetPack 6 (L4T R36.4) this was traced to Fast DDS's shared-memory transport. Switching the robot to Cyclone DDS fixes it:

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

**The controller pairs but the robot does not see it** (no ``/dev/input/js0``). Run ``bluetoothctl info <address>``: ``Bonded: no`` means the pairing was not saved, which happens when the computer's Bluetooth is not pairable, and the controller is then refused. Run ``bluetoothctl remove <address>`` and ``bluetoothctl pairable on``, put the controller back in pairing mode and pair again; ``Bonded: yes`` means it worked.

**``VESC n feedback stale ... holding all motors stopped``.** The driver stops the robot when any motor controller has not reported its speed for 250 ms, because the wheel controller would otherwise keep driving that wheel on a frozen speed reading. One such line at startup, before the first status frames arrive, is normal. If it repeats, check that VESC's power and CAN wiring.

**``CAN interface <name> not found``, followed by ``Error when connecting to robot``.** The ``device_port`` in the robot config does not exist. On CAN robots it should be ``rovercan``; confirm with ``ip -br link`` and see *Connection*.

**The controller and the robot stop when the robot leaves the WiFi, and come back when it returns.** By default Cyclone DDS ties all ROS traffic to the WiFi address, including messages between programs on the robot itself, so losing the WiFi stops the controller, the input manager and the driver from hearing each other. The log shows ``ddsi_udp_conn_write ... failed with retcode -1``. Re-run ``./setup_rover.sh --with-cyclone``: it sets up Cyclone so the robot's own traffic goes over loopback, which never goes away, and the WiFi and Ethernet ports are optional extras for laptops on the network. Tested on Humble and Jazzy with the WiFi off for 20 seconds: no gap. See the install scripts' README for details.

**The wheels keep turning for about a second after the service is stopped.** When the driver exits cleanly it sends a brake command to every motor controller, but only if systemd lets it shut down in order. The service installed by ``setup_rover.sh`` does this (``KillMode=mixed``, ``KillSignal=SIGINT``). On a robot installed before this change, re-run ``./setup_rover.sh --with-service``. A hard crash or ``kill -9`` cannot send the brake; to cover that case, set a *Timeout Brake Current* on each VESC in VESC Tool so the controllers brake rather than coast.

## Simulation with Gazebo
Our ROS2 packages now support simulations for all robots! The ``roverrobotics_gazebo`` package implements all of the simulation launches. You can launch your simulation using the following:
```bash
ros2 launch roverrobotics_gazebo <robot>_gazebo.launch.py
```
*Valid ``<robot>`` options are: ``2wd_rover, 4wd_rover, flipper, mini, mini_2wd, miti, miti_65, max, mega``*

The 2wd_rover and 4wd_rover replace the Rover Zero and Rover Pro since they have the same footprint. The 2wd_rover implements our chassis with two driven front wheels and two rear casters and the 4wd_rover implements our chassis with 4 driven wheels in a skid steer configuration.

Note: You have to install gazebo specifically for ROS. Our install script does not install gazebo. To install gazebo:
```sudo apt install ros-{DISTRO}-ros-gz```

## Adding Sensors (IMU, LiDAR, GPS, Camera)

Every sensor is added with the same five steps. Do them in order, and check each one before moving on:

1. **Install its software.** Done by the install script if you said yes to that sensor.
2. **Plug it in and check its name.** The install script gives each sensor a fixed name, such as ``/dev/bno055``, so it is found the same way after every reboot.
3. **Switch it on** in ``accessories.yaml`` by changing ``active: false`` to ``active: true``.
4. **Build and restart** (see *Step 4: Change a setting* above).
5. **Check its data** with ``ros2 topic hz <topic>``. A number of messages per second means it works.

All sensors are switched on in the same file:

```bash
nano ~/rover_workspace/src/roverrobotics_ros2/roverrobotics_driver/config/accessories.yaml
```

**If the IMU, the 2D LiDAR or the camera is switched on but unplugged or broken, the robot software stops and restarts every few seconds, and the robot will not drive.** This is deliberate, so a missing sensor is never silently ignored. To drive without it, set it back to ``active: false``, then build and restart.

### BNO055 IMU

The IMU measures how the robot turns and tilts. Navigation uses it to keep its heading accurate.

1. **Software.** Say yes to the IMU in ``setup_rover.sh``, or run ``./setup_rover.sh --with-imu`` again. This installs a ``bno055`` package with a fix for a startup timing problem, and with the service it also clears the IMU's serial port before every start.
2. **Plug it in and check its name:**
   ```bash
   ls -l /dev/bno055
   ```
   You should see a line ending in ``-> ttyUSB0`` (or another number). If you get *No such file or directory*, see *Sensor not found* below.
3. **Switch it on.** In ``accessories.yaml``, under ``bno055:``, set:
   ```yaml
   bno055:
     ros__parameters:
       active: true
       uart_port: "/dev/bno055"
   ```
4. **Build and restart.**
5. **Check it:**
   ```bash
   ros2 topic hz /imu/data
   ```
   You should see about 100 messages per second. Turn the robot by hand and watch the turn rate with ``ros2 topic echo /imu/data --field angular_velocity``.

The IMU's position on the robot is ``imu_link`` in the robot model (``roverrobotics_description/urdf/accessories/imu.urdf``). If you mount it somewhere else, update the position there.

### RPLIDAR S2 (2D LiDAR)

The 2D LiDAR measures distances all around the robot in one flat slice. Mapping (SLAM) and navigation use it to see walls and obstacles.

1. **Software.** Say yes to the LiDAR in ``setup_rover.sh``, or run ``./setup_rover.sh --with-lidar`` again.
2. **Plug it in and check its name:**
   ```bash
   ls -l /dev/rplidar
   ```
3. **Switch it on.** In ``accessories.yaml``, under ``rplidar:``, set:
   ```yaml
   rplidar:
     ros__parameters:
       active: true
       serial_port: "/dev/rplidar"
   ```
4. **Build and restart.** The LiDAR starts spinning.
5. **Check it:**
   ```bash
   ros2 topic hz /scan
   ```
   You should see about 10 scans per second (``scan_frequency`` in the same file).

The LiDAR's position on the robot is ``lidar_link`` (``roverrobotics_description/urdf/accessories/rplidar_s2.urdf``).

### SICK multiScan136 (3D LiDAR)

The 3D LiDAR sees in 3D, not only in one flat slice. It connects over Ethernet rather than USB.

1. **Software.** It is not installed by the install script. Build ``sick_scan_xd`` in your workspace by following [its instructions](https://github.com/SICKAG/sick_scan_xd) for ROS 2.
2. **Network.** Connect it to the robot's Ethernet port and give the robot's port an address on the scanner's network (the scanner's default is ``192.168.0.1``).
3. **Switch it on.** In ``accessories.yaml``, under ``multiscan:``, set ``active: true``, ``hostname`` to the scanner's address and ``udp_receiver_ip`` to the robot's own address on that network. Leave the other lines as they are.
4. **Build and restart.**
5. **Check it:**
   ```bash
   ros2 topic hz /sick/points
   ```

### GPS (jazzy branch)

The GPS is a **u-blox ZED-F9P** receiver on USB. Unlike the other sensors it runs as its own service, ``rover-ublox.service``, not with the robot software, so a GPS that drops out never stops the robot. It is supported on the ``jazzy`` branch.

1. **Software.** Run ``./setup_rover.sh --with-gps`` (or tick GPS in the installer). This installs the GPS driver, the service and its watchdog. The GPS stays off until step 3.
2. **Plug it in and check its name:**
   ```bash
   ls -l /dev/ublox-gps
   ```
   If the GPS is switched on, plugging it in starts it by itself.
3. **Switch it on.** In ``accessories.yaml``, under ``ublox_gps_node:``, set:
   ```yaml
   ublox_gps_node:
     ros__parameters:
       active: true
   ```
   The other settings in that block (update rate, satellite systems, ``frame_id: gps_link``) suit the ZED-F9P; leave them unless you know you need a change.
4. **Make sure your robot's model has the GPS on it.** The GPS position is published in the frame ``gps_link``, and navigation can only use it if the robot model says where that frame is. **None of the robot models include the GPS by default.** Open your robot's model, for example ``roverrobotics_description/urdf/miti.urdf`` (the MAX uses ``max_130.urdf`` or ``max_150.urdf``), and add this line under ``<!-- Part Includes - Payload, Sensors, Etc.. -->``, next to the other sensors:
   ```xml
   <xacro:include filename="$(find roverrobotics_description)/urdf/accessories/gps.urdf" />
   ```
   Then set where the GPS antenna sits on your robot in ``roverrobotics_description/urdf/accessories/gps.urdf``: the ``origin xyz`` of ``gps_to_payload`` is its position in metres from ``payload_link`` (forward, left, up). The model is loaded by the robot software, so this step needs the robot software restarted too (step 5).
5. **Build and restart** the GPS, and the robot software if you changed the model in step 4:
   ```bash
   cd ~/rover_workspace
   colcon build
   sudo systemctl restart rover-ublox
   sudo systemctl restart roverrobotics    # only after a model change
   ```
   Check that the robot model now has the GPS: ``ros2 run tf2_ros tf2_echo base_link gps_link`` should print a position, not an error.
6. **Check it:**
   ```bash
   ros2 topic hz /fix
   ```
   You should see about 8 messages per second. ``/fix`` is published even before the receiver has a satellite fix; check ``status`` in ``ros2 topic echo /fix`` and take the receiver outdoors with a clear view of the sky.

**Switching it off** is the same in reverse: ``active: false``, build, restart ``rover-ublox``. While it is off, ``systemctl status rover-ublox`` shows the service as *skipped*. That is normal.

What the service takes care of: it waits for a receiver that appears late at boot, starts the GPS as soon as the receiver is plugged in, power-cycles the receiver over USB before each start, retries a missing receiver every few seconds and then once a minute, and restarts the GPS if ``/fix`` goes silent. It does not restart for a missing satellite fix, which a restart cannot cure. Its log is ``journalctl -u rover-ublox``.

The GPS position on the robot is ``gps_link`` (``roverrobotics_description/urdf/accessories/gps.urdf``). The simulation also publishes a simulated ``/fix``.

### Intel RealSense camera

The install script installs the camera software when you say yes to RealSense. There are two ways to run the camera; use **one** of them, not both:

- **As its own service (recommended):** run ``./setup_rover.sh --with-rs-service``. The camera then starts at boot on its own and is restarted automatically if its frames stop. Leave ``realsense`` set to ``active: false`` in ``accessories.yaml``.
- **With the robot software:** set ``realsense`` to ``active: true`` in ``accessories.yaml``, then build and restart.

Check it with ``ros2 topic list | grep camera``. Plug the camera into a USB port on the computer itself, not the USB-C port used to flash a Jetson: that port does not work for devices.

### Sensor not found

If ``ls -l /dev/<name>`` says *No such file or directory*:

1. Unplug the sensor and plug it back in, then try again.
2. Run ``lsusb`` with the sensor plugged in and look for its line. If it is missing, the cable or the USB port is the problem.
3. If it is listed, the device rules may be missing: re-run ``./setup_rover.sh`` (it installs them by default), then unplug and replug the sensor.
4. Sensors from another manufacturer may have a different USB ID than the ones in the device rules. Find the ID with ``lsusb`` (the ``xxxx:yyyy`` after ``ID``) and add a line for it to ``udev/55-roverrobotics.rules`` in ``rover_install_scripts_ros2``, following the lines already there. Then install the rules:
   ```bash
   cd ~/rover_install_scripts_ros2/udev
   sudo cp 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
   If two sensors share the same ID, tell them apart with ``ATTRS{serial}=="..."`` (shown by ``lsusb -v``).

### Installing the sensor software by hand

Only needed if you did not use the install script:

```bash
cd ~/rover_workspace/src
git clone -b fix-startup-race https://github.com/ssharma0704/bno055.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/rover_workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

The ``bno055`` branch above is ``flynneva/bno055`` with the startup fix from its pull request 85.

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

## Release Notes — October 2026

This release follows the September release. It stops the robot when the controller goes out of range, fixes a driver freeze on ROS 2 Jazzy, corrects the battery current reading, moves the controller emergency stop to Cross, verifies the MAX 130 drive update on ROS 2 Jazzy, adds GPS to the jazzy branch and adds a beginner guide to this README. Every change listed here was tested on hardware before release.

### Highlights

- **The robot stops when the controller goes out of range.** Within 0.5 s of the controller's last report, instead of driving on for up to 15 s.
- **No driver freeze on ROS 2 Jazzy.** The driver now runs on the single-threaded executor.
- **MAX 130 drive update verified on ROS 2 Jazzy,** with the same stand and ground results as on Humble. See *MAX 130 drive update* under *Release Notes — September 2026*.
- **GPS on the jazzy branch,** a u-blox ZED-F9P run by its own service.
- **Beginner guide** for installing, driving and changing a setting without prior ROS knowledge.

### Bug fixes

- **Fixed ``battery_status.current`` decoding.** The motor controller's input current was read as unsigned with ten times too small a scale; it is now signed and correctly scaled, so it reads negative while the motor regenerates.
- **Removed false charging reports.** The CAN robots derived CHARGING from that motor current, which a charger never passes through; ``power_supply_status`` is now UNKNOWN on them.
- **Fixed the robot driving on when the controller goes out of range.** The joystick driver kept repeating the last stick position after the Bluetooth link stalled, for 14.6 s in a test at 1.25 m/s, until Linux declared the controller disconnected. The input manager now stops the robot 0.5 s after the controller's reports stop, and waits for centred sticks before driving again. See *When the controller goes out of range*.
- **Fixed the driver freezing on ROS 2 Jazzy.** Jazzy's multi-threaded executor can permanently stop running a callback group ([ros2/rclcpp#3240](https://github.com/ros2/rclcpp/issues/3240)); the driver then stays alive but publishes nothing and ignores ``/cmd_vel``. It happened within a minute on every start under ``rmw_zenoh``, and is rarer with Fast DDS. The driver now uses the single-threaded executor. All its callbacks were already in one callback group, which runs them one at a time, so nothing ran in parallel before either and the robot drives the same.

### Improvements

- **GPS on the jazzy branch.** A u-blox ZED-F9P switched on and off in ``accessories.yaml`` (``ublox_gps_node``) and run by its own service, so a GPS that drops out never stops the robot. See *GPS (jazzy branch)* under *Adding Sensors*.
- **Beginner guide.** *New here? Start with this* walks through installing, driving, everyday commands and changing a setting without prior ROS knowledge, and *Adding Sensors* gives step-by-step setup and checks for the BNO055 IMU, RPLIDAR S2, SICK multiScan136, GPS and RealSense.
- **Package version 1.1.0** on every package, with a current maintainer.
- **Removed the unused ``diagnostics_frequency`` setting** from every robot config.
- **No build warnings on Jazzy** from the deprecated ``rcppmath`` rolling-mean name; Humble keeps the name it supports.

### Changes to be aware of

- **The controller emergency stop moved to Cross (✕),** and Circle (○) now releases it; Triangle no longer does anything. Tell anyone who drives the robots.
- **``power_supply_status`` is UNKNOWN on the CAN robots,** instead of FULL, DISCHARGING or NOT_CHARGING derived from one motor's current. Monitoring that keyed on those values should use ``percentage`` and ``voltage``.

### Known issues

- Charging is not reported on the CAN robots, which have no battery current sensor: ``power_supply_status`` is UNKNOWN, and ``battery_status.current`` is one motor controller's input current, not the battery's (see the ``battery_status`` section).
- With Fast DDS, or with Cyclone DDS set up by hand, losing the WiFi can stop all ROS traffic on the robot until the WiFi returns. Cyclone DDS set up by ``setup_rover.sh --with-cyclone`` does not have this problem; Fast DDS has not been tested for it.

### Development timeline

A dated record of the work in this release, for reference.

| Date | Work | Verified on |
| --- | --- | --- |
| 2026-10-01 | Verified the MAX 130 drive update on ROS 2 Jazzy: stand speed sweep and pivot, ground speed hold with tape measure, pivots, arcs, forward-to-pivot, and two recorded controller drives, one with the commands logged. Added the beginner guide and the step-by-step sensor setup to this README. | MAX 130 on an Orin Nano, JetPack 7: stand 99.6 to 100.2% at 0.1 to 0.8 m/s; tape 3.02 m for 3.0 m commanded, odometry −0.5%; pivot 102%; arc 99%; stops after a quick stick release 0.46 to 0.86 s from up to 1.87 m/s; top speed held at the 1.875 m/s controller limit |
| 2026-10-01 | Fixed the ``battery_status.current`` decoding (signed, correct scale) and set ``power_supply_status`` to UNKNOWN on the CAN robots, which cannot detect charging. Removed the unused ``diagnostics_frequency`` setting from every robot config. Set every package to version 1.1.0 with a current maintainer. Removed the build warnings on Jazzy from the deprecated ``rcppmath`` rolling-mean name, keeping Humble on the name it supports. Moved the controller emergency stop to Cross, with Circle to release it. | MAX 130 on a stand, ROS 2 Jazzy: published ``current`` equal to the motor controller's own report, +0.4 to +0.5 A driving and −0.1 A while braking (the old decoding turned that −0.1 A into 655 A); ``power_supply_status`` UNKNOWN throughout. Controller emergency stop on a PS4 controller: Cross engaged and Circle released it on every press, within 10 ms. Driver builds without warnings on ROS 2 Humble and Jazzy |
| 2026-10-02 | Added the GPS (jazzy branch) and verified on hardware: a cold power-up with the driver answering its first command, a stop when motor controllers go silent, the wheel revolution count, the turn settling after a pivot, and the robot software surviving a loss of WiFi with Cyclone DDS set up by the install scripts. | MAX 130 and MITI on ROS 2 Jazzy, Orin Nano: driver moving the wheels 0.14 to 0.16 s after its first command after power-up; with two motor controllers disconnected all wheels held stopped within 0.75 s; tachometer 10.00 to 10.02 revolutions for 10 counted by eye; 0.55 degree settle after a pivot; no gap in controller or odometry messages with the WiFi off for 20 s (ROS 2 Humble too) |
| 2026-10-05 | Reproduced the ROS 2 Jazzy executor freeze (ros2/rclcpp#3240) and fixed it by moving the driver to the single-threaded executor. | MITI on a stand, Orin Nano, ROS 2 Jazzy: with commands at 100 Hz, the multi-threaded driver froze in 4 of 4 runs under rmw_zenoh, the single-threaded driver in none; Fast DDS and Cyclone DDS unaffected either way; the same scripted drive with both executors gave the same starts, stops (0.14 to 0.34 s, no reverse duty), steady speed (100.7 to 101.5%) and currents |
| 2026-10-06 | Measured the robot driving on with a controller out of range and added the controller link check to the input manager. Added ``pairable on`` to the controller pairing steps, after a PS5 paired without its pairing being saved and was refused. | MITI on a stand, Orin Nano, ROS 2 Jazzy: before, 14.6 s of driving at 1.25 m/s after the last controller report (PS4); after, the wheels stopped 0.84 to 0.90 s after the last report with a PS4 and a PS5 going out of Bluetooth range, and 0.72 s after a PS5 was unplugged from USB (0.5 s detection plus braking), with no stick command passed on while the link was down and no false stop in idle pauses of up to 149 s |

---

## Release Notes — September 2026

This release is a reliability and driving-quality update for every CAN robot (Mini, MITI, MAX and MEGA). It makes wheel speed and odometry correct, fixes a runaway-wheel defect, makes stops smooth and battery-safe, adds a controller emergency stop and battery calibration, and brings the Humble and Jazzy branches to the same driver code. The MITI also gains accurate low-speed driving with feedforward wheel control. Every change listed here was tested on hardware before release.

### Highlights

- **Correct speed and odometry on every robot.** Wheel speed was under-reported by 10% on the Mini and MITI and by 40% on the MAX and MEGA. It is now exact, and odometry measures true distance.
- **Accurate, smooth low-speed driving on the MITI.** Low-speed wheel speed is now read correctly, and feedforward wheel control gives launches without overshoot, faster pivots and the quietest drive of all settings tested. See *MITI drive update* below.
- **Smooth, battery-safe stopping.** A new braking band removes the jolt at the end of a stop and keeps regenerative braking within what the battery accepts, including from full speed.
- **Emergency stop on the controller.** Circle stops the robot; Triangle resets it. Works on PS4 and PS5 controllers. The buttons moved to Cross and Circle in October; see *Release Notes — October 2026*.
- **Retuned motor control.** New PID gains for the Mini, MITI and MAX, tuned on hardware with the corrected speed feedback.
- **Battery percentage from a calibrated voltage,** configurable per pack, with 0% set above the battery protection cut-off.
- **One driver for Humble and Jazzy.** Both branches carry identical driver code and configs.

### MITI drive update

These changes are opt-in per robot. They are enabled on the MITI, and can be enabled on other robots after the same calibration.

- **Low-speed speed reading.** Below their *Hall Interpolation ERPM* setting (default 500) the VESCs reported about half the real wheel speed, so the MITI drove up to 2.2 times too fast at low speed and odometry came up 32% short. With the setting at 50, odometry is within 0.5% of a tape measure at 0.2 m/s. See *Speed feedback at low speed*.
- **Feedforward wheel control** with battery-voltage compensation, a turn assist, a launch hold and low-speed handling. On the ground it held steady speed to about 1% of the command and launched with at most 2% overshoot. See *Feedforward and launch control*.
- **Turn rates that match the command.** ``wheel_base`` is now the effective track width, 0.60 m, so pivots and arcs reach about 90% of the commanded rate (66% before) and wheel odometry yaw matches the IMU. See *Kinematics*.
- **Required on every MITI:** set *Hall Interpolation ERPM* to 50 and enable CAN status 5 on each VESC in VESC Tool, then write the configuration. The new MITI gains are tuned for this setting.

### MAX 130 drive update

These changes are opt-in and enabled only in ``max_130_config.yaml``; the MAX 150 and every other robot drive as before. All values were measured on a MAX 130 without payload.

- **Calibrated geometry.** Effective wheel radius 0.155 and effective track width 0.90 m: the robot now drives 3.015 m on the tape for 3.0 m commanded, and pivots and arcs reach 98 to 103% of the commanded turn rate (53% and 49% before). See *Kinematics*.
- **Feedforward with lower gains.** Calibrated at 40.3 V; steady speed at 100% of the command, with about half the duty ripple of the PID alone. See *Feedforward and launch control*.
- **Arcs and pivots that hold their rate.** The PID correction no longer leaks away (``ff_correction_decay``), and a stale correction is cleared at once (``ff_correction_release``): no forward creep before a pivot and no speed dip after a turn.
- **Firm, consistent stops.** ``brake_band_duty`` 0.20 with ``brake_momentum_carry``: every stop now follows the band, about 0.9 s from 2.2 m/s, where before some stops were soft (1.8 s) and others hard. See *Stopping and braking*.
- **Gentle starts and turns.** ``max_linear_acceleration`` 1.5 and ``max_angular_acceleration`` 4.0: launch current fell from about 30 A to 16 A (median) and pivot-start current from 37 A to 21 A. Stops are unchanged. See *Velocity handling*.
- **Controller limits.** Full stick gives 1.25 m/s and 1.25 rad/s, and the D-pad raises both to at most 1.875 in two presses. See ``/cmd_vel`` under *Published Topics and Units*.
- **Required:** *Hall Interpolation ERPM* 50 on every VESC, and ``max.launch.py`` and ``max_teleop.launch.py`` both pointing at ``max_130_config.yaml``.
- **Same on Jazzy (October).** The Jazzy branch carries identical MAX 130 code and settings, and was verified on a MAX 130 with an Orin Nano on JetPack 7: the same stand and ground results as on Humble.

### What's new

- **Braking band** (``brake_band_duty``, ``brake_band_rpm``, ``rpm_per_duty``). Bounds how hard the wheel controller may brake a rolling wheel. Enabled on the MAX 130 and MAX 150; off by default elsewhere. See *Stopping and braking*.
- **Feedforward** (``ff_rpm_per_duty``, ``ff_static_duty``, ``ff_turn_duty``, ``ff_calibration_voltage``, ``low_speed_trust_rpm``) and **speed smoothing for the PID** (``wheel_speed_filter``). Enabled on the MITI. See *Feedforward and launch control*.
- **Tachometer speed source** (``use_tachometer_speed``), an optional fallback that reads wheel speed from the VESC tachometer (CAN status 5).
- **Battery calibration** (``battery_cells``, ``battery_max_cell_voltage``, ``battery_min_cell_voltage``, ``battery_voltage_multiplier``) and ``power_supply_status`` on ``battery_status`` (Rover Pro only; UNKNOWN on the CAN robots). See *Battery*.
- **Configurable rest release** (``rest_wheel_rpm``). The speed below which a stopped wheel is released, previously fixed in code.
- **Optional release hold** (``release_hold_s``). Keeps the motors braked briefly after the wheels read zero; off by default.
- **Controller emergency stop.** ``topics.yaml`` maps Circle to ``/soft_estop/trigger`` and Triangle to ``/soft_estop/reset`` (Cross and Circle since October). The input manager gained a button-to-``std_msgs/Bool`` topic type to support it.
- **Estop status topic.** ``/soft_estop/status`` publishes the current estop state as a latched ``std_msgs/Bool``.
- **Per-wheel joint states.** ``/joint_states`` now publishes each wheel's angle and true angular velocity.
- **Odometry reset.** Publish to ``/roverrobotics_driver/reset_odometry`` to zero the pose without restarting the driver.
- **Unit serial number.** A new ``serial_number`` parameter is published once, latched, on ``rover_<robot_type>/serial_number``.
- **Per-wheel trims** (``wheel_trim_fl`` / ``fr`` / ``rl`` / ``rr``) to balance a wheel that runs fast or slow.
- **Pose covariance parameters** (``pose_linear_covariance``, ``pose_yaw_covariance``) so sensor-fusion nodes weight wheel odometry correctly.
- **Command timeout and deceleration limit** (``cmd_vel_timeout_sec``, ``max_velocity_step``). The robot stops by itself if its velocity publisher goes quiet.
- **Controller speed overrides.** The PS5 launch accepts ``lin_increment``, ``ang_increment``, ``max_lin_speed``, ``max_ang_speed``, ``start_lin_throttle`` and ``start_ang_throttle`` so a robot can adjust its teleop feel without editing shared files.
- **Feedforward correction options** (``ff_correction_decay``, ``ff_correction_release``). Enabled on the MAX 130. See *Feedforward and launch control*.
- **Gentle start limits** (``max_linear_acceleration``, ``max_angular_acceleration``). Limit how fast forward speed and turn rate build up, without changing stops. Enabled on the MAX 130. See *Velocity handling*.
- **Braking momentum carry** (``brake_momentum_carry``). Keeps a stop on the braking band when the robot is released while still speeding up. Enabled on the MAX 130. See *Stopping and braking*.
- **Per-robot controller limits.** ``max_teleop.launch.py`` reads a ``joy_manager`` block from the MAX config, as ``pro_teleop.launch.py`` does for the Pro. See ``/cmd_vel`` under *Published Topics and Units*.
- **Launch supervision.** Every robot launch ends when the driver or an accessory node exits, so the service restarts the whole stack cleanly instead of respawning one node in a half-working stack.

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
- **Fixed the wheels running on after the driver stops.** The driver's last command was a driving command, so the motor controllers kept driving for about a second and then coasted. It now sends a brake to all four controllers on exit: on a stand at 1 m/s the wheels stopped in 0.26 s instead of 1.7 s.
- **Fixed ``estop_state`` being ignored.** The driver logged "Estop state is currently active" but the robot still drove. The configured state is now applied at startup.
- **Fixed a robot running on stale feedback.** If one motor controller stopped reporting, its wheel was controlled from a frozen speed reading. The driver now stops the robot when any controller is silent for 250 ms.
- **Fixed crash loops from configuration.** A whole number such as ``8`` instead of ``8.0`` in a robot config made the driver crash and restart continuously; numeric parameters now accept both.
- **Fixed a wrong CAN interface being reported as connected.** A ``device_port`` that does not exist is now reported as an error instead of a silent "Connected" with no motor commands sent.
- **Fixed ``/robot_info`` requests.** Requests on ``robot_info_request_topic`` now work; previously the estop reset triggered it instead.
- **Removed log flooding.** The command-timeout warning was printed every 2 seconds while idle (about 43,000 lines a day); it is now logged once per stop.
- **Removed the unused trim file on CAN robots.** ``/trim_event`` and ``~/robot.config`` had no effect on the Mini, MITI, MAX and MEGA wheel control; a malformed file could crash the driver, and building its path wrote into the ``HOME`` environment string. The per-wheel ``wheel_trim_*`` parameters replace it. The Rover Pro keeps its trim.
- **Hardened the VESC and CAN layer.** An unknown command type no longer terminates the driver, CAN write failures are reported, and the ``SET_CURRENT`` command is scaled in milliamps as the VESC expects.

### Improvements

- **Stable CAN interface naming.** All CAN configs now use ``rovercan``, a fixed name given to the USB-CAN adapter by a udev rule, so the driver can no longer bind to an unused onboard CAN controller after a reboot.
- **PS5 is the default controller** in every teleop launch. PS4 remains fully supported.
- **Deceleration tuned on hardware.** ``max_velocity_step`` is 0.75 on every robot; the earlier 0.05 made the robot coast after the stick was released.
- **Clearer startup logging.** The driver logs its gear ratio, pole pairs, control mode, braking settings and serial number at startup, and warns about invalid settings.

### Changes to be aware of

- **Retune custom gains.** Correcting the speed feedback changed the effective loop gain by about +11% on the Mini and MITI and +67% on the MAX and MEGA. Gains tuned against the old feedback should be retuned; the shipped gains already are.
- **MITI: set *Hall Interpolation ERPM* to 50 before using the new gains.** Do not combine it with the previous MITI gains (P 0.0007, D 0.00009): on a stand they produced large current swings.
- **MITI: ``wheel_base`` is now 0.60,** the effective track width rather than the 0.387 m between wheel centers. It depends on tyres and floor, so navigation should still fuse IMU yaw.
- **MAX 6.5 inch and 10 inch variants are no longer supported.** Their configs and URDFs were removed. The MAX is supported with 13 inch and 15 inch wheels.
- **``device_port`` is now ``rovercan``.** Manual installs must install the udev rule described under *Connection*.
- **Re-run ``setup_rover.sh --with-service`` on existing robots.** The brake-on-exit needs the service to stop gracefully (``KillMode=mixed``, ``KillSignal=SIGINT``), which the updated install script sets; see *Troubleshooting*. The service also restarts the stack when a node exits: a manual ``ros2 launch`` now ends instead of respawning the driver.
- **CAN robots now refuse to drive on stale feedback.** A motor controller that stops reporting brings the robot to a stop instead of letting it drive on.
- **Battery percentage is now configurable per pack** (see *Battery*). With the defaults a 10-cell pack still reads 0% at 34 V and 100% at 42 V. The MITI applies a 1.025 correction to the reported voltage, measured against a meter, so it reads slightly higher than before at the same pack voltage.
- **With the BNO055 enabled, use a ``bno055`` package with the startup-retry fix** (flynneva/bno055 pull request 85). Without it the IMU node can exit at boot before its serial port appears, and the stack restarts until the port is ready.
- **MAX 130: new gains assume feedforward.** P 0.0005 / D 0.000025 are tuned together with the MAX 130 feedforward. If you turn feedforward off, go back to P 0.0012 / D 0.00006.
- **MAX 130: the controller is slower by default.** Full stick is now 1.25 m/s and 1.25 rad/s (1.25 m/s and 2.5 rad/s before), with a ceiling of 1.875 on both.
- **Stopping from 160 to 215 rpm takes 0.1 to 0.25 s longer on the MAX** with the braking band enabled, and stops from full speed take about 2.1 to 2.9 s. This is the cost of keeping regenerative braking within what the battery accepts.

### Known issues

- An emergency stop at full speed brakes as hard as the motors allow and briefly raised the bus to about 55 V in testing. It is safe to use, but it should not be the routine way to stop at top speed.
- On some JetPack 6 systems, Fast DDS can stop delivering messages between processes shortly after start. Use Cyclone DDS as described under *Troubleshooting*.
- The braking band's default values were measured on a MAX 130. Confirm ``rpm_per_duty`` on the first MAX 150 before relying on it for hard stops.
- Below about 0.07 m/s on the MITI the VESC speed reading is still unreliable (too few hall edges), so a small bump can remain when starting at a crawl.
- Feedforward is calibrated for the MITI and the MAX 130 only. Other robots use the PID alone until calibrated.
- The MAX 130 values were measured without payload. With a heavy payload, check the feedforward and the stop distance.
- On the MAX 130, a hard turn while driving at 2.2 m/s or faster can still draw 40 to 50 A for a moment as the inner wheels are braked through zero. The controller limits keep the robot below that speed from the pad.
- The controller's D-pad multiplier is not capped: pressing past ``max_lin_speed`` or ``max_ang_speed`` makes partial stick faster until the D-pad is pressed back down.

### Development timeline

A dated record of the work in this release, for reference.

| Date | Work | Verified on |
| --- | --- | --- |
| 2026-09-16 | Fixed the runaway-wheel defect (out-of-range motor command array). Added clean driver shutdown: thread stop and join, CAN read timeout, virtual destructors, and a safe default in the VESC message parser. | Drive-tested on an AGX Thor test rover |
| 2026-09-17 | Corrected wheel speed and odometry scale with the new ``motor_pole_pairs`` parameter. Standardised CAN naming on ``rovercan``. Removed the MAX 6.5 and 10 inch variants. Made PS5 the default controller. Set ``max_velocity_step`` to 0.75 on every robot. Added the command timeout, per-wheel trims, serial number and pose covariance parameters. | Odometry ratio 1.000 on a MITI and a MEGA; full change set validated on a freshly installed MITI |
| 2026-09-21 | Brought up a Mini on an Orin Nano Super with ROS 2 Jazzy from this release; CAN communication verified. | Mini |
| 2026-09-22 | Tuned the Mini's PID gains (P 0.0008, D 0.00006) and verified the PS5 controller over Bluetooth. Tuned the MAX 130 with a 50 to 70 lb payload across nine recorded runs (P 0.0012, D 0.00006). Identified that the remaining stop jolt was not caused by the gains. | Mini; MAX 130 |
| 2026-09-23 | Traced the stop jolt to reverse duty sent to still-rolling wheels, and found that hard stops from high speed were tripping the battery protection. Built a simulator from recorded CAN data to evaluate fixes, rejected a first design that failed at speed, and developed the braking band. Added the controller emergency stop. | Stand tests and ground tests on the MAX 130 with payload |
| 2026-09-24 | Traced intermittent loss of controller input to the Fast DDS shared-memory transport and moved the test robot to Cyclone DDS. Tested the emergency stop from speed. Finalised and cleaned up the driver code, applied the MAX 130 settings to the MAX 150, merged the release into the Humble and Jazzy branches, and updated this documentation. Verified the remaining open defects one by one on a stand and fixed them: brake on driver exit, ``estop_state`` at startup, the ``/soft_estop/status`` topic, stale-feedback stop, configuration crash loops, CAN interface errors, ``/robot_info`` requests, log flooding, the unused trim file and the VESC codec. | MAX 130 on a stand and on the ground |
| 2026-09-25 | Traced the MITI's low-speed speed and odometry error to the VESC *Hall Interpolation ERPM* setting, confirmed with the VESC tachometer and by counting wheel revolutions, and set it to 50. Added the tachometer speed source. Measured the effective track width (``wheel_base`` 0.60). Developed feedforward with a turn assist, launch hold and low-speed handling, fixing each defect found on the stand, and calibrated it on the ground against true wheel speed. | MITI: odometry within 0.5% of tape at 0.2 m/s; pivots 91% and arcs 90% of commanded turn rate; launches at 0.3 and 0.6 m/s without overshoot |
| 2026-09-28 | Compared feedforward against the PID alone, with low gains and with the previous MITI gains, on a stand and in 12 ground runs with the robot reset between runs; feedforward gave the steadiest drive and the smallest launch overshoot. Added battery-voltage compensation. Confirmed steady speed at about 101% of the command in a 10 s hold. | MITI: straight-line odometry −1.0% with feedforward, −1.2% with low gains, −8.7% with the previous gains; launch peak 102% |
| 2026-09-29 | Added battery calibration, set 0% to 3.4 V per cell, and verified the release on two MITIs, including a cold power cycle. | Two MITIs |
| 2026-09-29 | Calibrated the MAX 130 without payload: *Hall Interpolation ERPM* 50, effective wheel radius 0.155 and track width 0.90, and feedforward from stand and ground holds in both directions. | MAX 130: tape 3.015 m for 3.0 m commanded, odometry −0.5%; pivot 104%, arc 98% |
| 2026-09-30 | Lowered the MAX 130 gains for feedforward and added ``ff_correction_decay``. Traced the forward creep before a pivot to a correction frozen by the launch hold and added ``ff_correction_release``. Traced inconsistent stops to the braking band handing a still-accelerating wheel to the PID; set ``brake_momentum_carry`` and ``brake_band_duty`` 0.20. Added the gentle start limits, then moved them from the measured speed to the command after a speed dip while weaving, and extended ``ff_correction_release`` to corrections left over from a turn. Added the MAX controller limits. Each change was tested on a stand and then in a recorded pad drive. | MAX 130: arc 103%, pivot 102%; forward drift before a pivot 0.16 → 0.08 m; 35 of 35 stops on the band; launch current median 31 → 16 A; weaving speed dip median 20% → 4%; pivot-start current median 37 → 21 A |
