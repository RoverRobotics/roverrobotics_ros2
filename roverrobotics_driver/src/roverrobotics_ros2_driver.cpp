#include <cmath>
#include "roverrobotics_ros2_driver.hpp"
using namespace RoverRobotics;
#include <iostream>

double inMin = 750.0;
double inMax = 970.0;
double outMin = 0.0;
double outMax = 100.0;
double mapValue(double x, double inMin, double inMax, double outMin, double outMax)
{
  if (x<inMin){
    return 0.0;
  }
  else
  {
    return outMin + (x-inMin)*(outMax-outMin)/(inMax-inMin);
  }
}

RobotDriver::RobotDriver() : Node("roverrobotics", rclcpp::NodeOptions().use_intra_process_comms(false)), linear_accumulator_(10),
  angular_accumulator_(10){
  RCLCPP_INFO(get_logger(), "Starting Rover Driver node");
  // Robot
  robot_status_topic_ =
      declare_parameter("robot_status_topic", ROBOT_STATUS_TOPIC_DEFAULT_);
  robot_status_frequency_ = declare_number_("robot_status_frequency",
                                              ROBOT_STATUS_FREQUENCY_DEFAULT_);
  robot_info_request_topic_ = declare_parameter(
      "robot_info_request_topic", ROBOT_INFO_REQUEST_TOPIC_DEFAULT_);
  robot_info_topic_ =
      declare_parameter("robot_info_topic", ROBOT_INFO_TOPIC_DEFAULT_);
  robot_type_ = declare_parameter("robot_type", ROBOT_TYPE_DEFAULT_);
  device_port_ = declare_parameter("device_port", DEVICE_PORT_DEFAULT_);
  comm_type_ = declare_parameter("comm_type", COMM_TYPE_DEFAULT_);
  wheel_radius_ = declare_number_("wheel_radius", WHEEL_RADIUS_DEFAULT_);
  wheel_base_ = declare_number_("wheel_base", WHEEL_BASE_DEFAULT_);
  robot_length_ = declare_number_("robot_length", ROBOT_LENGTH_DEFAULT_);
  gear_ratio_ = declare_number_("gear_ratio", GEAR_RATIO_DEFAULT_);
  motor_pole_pairs_ = declare_number_("motor_pole_pairs", MOTOR_POLE_PAIRS_DEFAULT_);
  serial_number_ = declare_parameter("serial_number", SERIAL_NUMBER_DEFAULT_);
  wheel_trim_fl_ = declare_number_("wheel_trim_fl", WHEEL_TRIM_FL_DEFAULT_);
  wheel_trim_fr_ = declare_number_("wheel_trim_fr", WHEEL_TRIM_FR_DEFAULT_);
  wheel_trim_rl_ = declare_number_("wheel_trim_rl", WHEEL_TRIM_RL_DEFAULT_);
  wheel_trim_rr_ = declare_number_("wheel_trim_rr", WHEEL_TRIM_RR_DEFAULT_);

  speed_topic_ = declare_parameter("speed_topic", SPEED_TOPIC_DEFAULT_);
  estop_trigger_topic_ =
      declare_parameter("estop_trigger_topic", ESTOP_TRIGGER_TOPIC_DEFAULT_);
  estop_reset_topic_ =
      declare_parameter("estop_reset_topic", ESTOP_RESET_TOPIC_DEFAULT_);
  estop_status_topic_ =
      declare_parameter("estop_status_topic", ESTOP_STATUS_TOPIC_DEFAULT_);
  trim_topic_ = declare_parameter("trim_topic", TRIM_TOPIC_DEFAULT_);
  estop_state_ = declare_parameter("estop_state", ESTOP_STATE_DEFAULT_);
  control_mode_name_ = declare_parameter("control_mode", CONTROL_MODE_DEFAULT_);
  linear_top_speed_ =
      declare_number_("linear_top_speed", LINEAR_TOP_SPEED_DEFAULT_);
  angular_top_speed_ =
      declare_number_("angular_top_speed", ANGULAR_TOP_SPEED_DEFAULT_);
  float pi_p_ = declare_number_("motor_control_p_gain", PID_P_DEFAULT_);
  float pi_i_ = declare_number_("motor_control_i_gain", PID_I_DEFAULT_);
  float pi_d_ = declare_number_("motor_control_d_gain", PID_D_DEFAULT_);
  linear_covariance = declare_number_("linear_covariance", LIN_COVAR_DEFAULT);
  yaw_covariance = declare_number_("yaw_covariance", YAW_COVAR_DEFAULT);
  pose_linear_covariance =
      declare_number_("pose_linear_covariance", POSE_LIN_COVAR_DEFAULT);
  pose_yaw_covariance =
      declare_number_("pose_yaw_covariance", POSE_YAW_COVAR_DEFAULT);

  max_velocity_step_ = declare_number_("max_velocity_step", MAX_VELOCITY_STEP_DEFAULT_);
  cmd_vel_timeout_sec_ = declare_number_("cmd_vel_timeout_sec", CMD_VEL_TIMEOUT_DEFAULT_);
  last_cmd_time_ = steady_clock_.now();
  double rest_wheel_rpm = declare_number_("rest_wheel_rpm", REST_WHEEL_RPM_DEFAULT_);
  if (!std::isfinite(rest_wheel_rpm) || rest_wheel_rpm <= 0.0 || rest_wheel_rpm > 30.0) {
    RCLCPP_ERROR(get_logger(), "rest_wheel_rpm %f out of (0, 30], using %.1f",
                 rest_wheel_rpm, REST_WHEEL_RPM_DEFAULT_);
    rest_wheel_rpm = REST_WHEEL_RPM_DEFAULT_;
  }
  rest_wheel_rpm_ = static_cast<float>(rest_wheel_rpm);
  double brake_band_duty = declare_number_("brake_band_duty", BRAKE_BAND_DUTY_DEFAULT_);
  double brake_band_rpm = declare_number_("brake_band_rpm", BRAKE_BAND_RPM_DEFAULT_);
  double rpm_per_duty = declare_number_("rpm_per_duty", RPM_PER_DUTY_DEFAULT_);
  if (!std::isfinite(brake_band_duty) || brake_band_duty < 0.0 || brake_band_duty > 0.5 ||
      !std::isfinite(brake_band_rpm) || brake_band_rpm < 0.0 ||
      !std::isfinite(rpm_per_duty) || rpm_per_duty <= 0.0 ||
      (brake_band_duty > 0.0 &&
       (rpm_per_duty < RPM_PER_DUTY_MIN_ || rpm_per_duty > RPM_PER_DUTY_MAX_))) {
    RCLCPP_ERROR(get_logger(), "brake band %f/%f/%f invalid, braking band off",
                 brake_band_duty, brake_band_rpm, rpm_per_duty);
    brake_band_duty = BRAKE_BAND_DUTY_DEFAULT_;
    brake_band_rpm = BRAKE_BAND_RPM_DEFAULT_;
    rpm_per_duty = RPM_PER_DUTY_DEFAULT_;
  }
  brake_band_duty_ = static_cast<float>(brake_band_duty);
  brake_band_rpm_ = static_cast<float>(brake_band_rpm);
  rpm_per_duty_ = static_cast<float>(rpm_per_duty);
  double release_hold_s = declare_number_("release_hold_s", RELEASE_HOLD_S_DEFAULT_);
  if (!std::isfinite(release_hold_s) || release_hold_s < 0.0 || release_hold_s > 1.0) {
    RCLCPP_ERROR(get_logger(), "release_hold_s %f out of [0, 1], using %.1f",
                 release_hold_s, RELEASE_HOLD_S_DEFAULT_);
    release_hold_s = RELEASE_HOLD_S_DEFAULT_;
  }
  release_hold_s_ = static_cast<float>(release_hold_s);
  
  linear_accumulator_ = RollingMeanAccumulator(10);
  angular_accumulator_ = RollingMeanAccumulator(10);
  // Odom
  pub_odom_tf_ = declare_parameter("publish_tf", PUB_ODOM_TF_DEFAULT_);
  odom_topic_ = declare_parameter("odom_topic", "/odom_raw");
  odometry_frequency_ =
      declare_number_("odometry_frequency", ROBOT_ODOM_FREQUENCY_DEFAULT_);
  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  odom_child_frame_id_ =
      declare_parameter("odom_child_frame_id", "base_link");
  // Angular Scaling params
  angular_scaling_params_.a_coef =
      declare_number_("angular_a_coef", ANGULAR_SCALING_A_DEFAULT_);
  angular_scaling_params_.b_coef =
      declare_number_("angular_b_coef", ANGULAR_SCALING_B_DEFAULT_);
  angular_scaling_params_.c_coef =
      declare_number_("angular_c_coef", ANGULAR_SCALING_C_DEFAULT_);
  angular_scaling_params_.min_scale_val =
      declare_number_("angular_min_scale", ANGULAR_SCALING_MIN_DEFAULT_);
  angular_scaling_params_.max_scale_val =
      declare_number_("angular_max_scale", ANGULAR_SCALING_MAX_DEFAULT_);
  // Finished getting all parameters
  RCLCPP_INFO(get_logger(),
              "Robot type is Rover %s over %s", robot_type_.c_str(), comm_type_.c_str());
  RCLCPP_INFO(get_logger(), "Receiving velocity command from %s", speed_topic_.c_str());
  if (estop_state_)
    RCLCPP_INFO(get_logger(), "Estop state is currently active");
  else
    RCLCPP_INFO(get_logger(), "Estop state is currently inactive");

  RCLCPP_INFO(get_logger(), "Receiving Estop activation at %s", estop_trigger_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Receiving Estop deactivation at %s", estop_reset_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing Estop status at %s", estop_status_topic_.c_str());
 
  
  // Init Sub
  speed_command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      speed_topic_, rclcpp::QoS(1),
      [=](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        velocity_event_callback(msg);
      });
  trim_event_subscriber_ = create_subscription<std_msgs::msg::Float32>(
      trim_topic_, rclcpp::QoS(3),
      [=](std_msgs::msg::Float32::ConstSharedPtr msg) {
        trim_event_callback(msg);
      });
  estop_trigger_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      estop_trigger_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_trigger_event_callback(msg);
      });
  estop_reset_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      estop_reset_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_reset_event_callback(msg);
      });
  reset_odometry_subscriber_ = create_subscription<std_msgs::msg::Empty>(
      "~/reset_odometry", rclcpp::QoS(1),
      [=](std_msgs::msg::Empty::ConstSharedPtr) {
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        theta_ = 0.0;
        RCLCPP_INFO(get_logger(), "Odometry pose reset to zero");
      });

  robot_info__request_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      robot_info_request_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        robot_info_request_callback(msg);
      });

  // Init Pub

  robot_info_publisher = create_publisher<std_msgs::msg::Float32MultiArray>(
      robot_info_topic_, rclcpp::QoS(32));
  robot_status_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      robot_status_topic_, rclcpp::QoS(31));
  battery_soc_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      "rover_" + robot_type_ + "/battery_status", rclcpp::QoS(10));
  if (pub_odom_tf_) {
     RCLCPP_INFO(get_logger(),
                "Publishing Robot TF on %s at %.2Fhz", odom_topic_.c_str(),
                odometry_frequency_);
  }
  odom_tf_pub = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  /* transient_local so a subscriber that starts later still receives it */
  estop_status_publisher_ = create_publisher<std_msgs::msg::Bool>(
      estop_status_topic_, rclcpp::QoS(1).transient_local());
  serial_number_publisher_ = create_publisher<std_msgs::msg::String>(
      "rover_" + robot_type_ + "/serial_number",
      rclcpp::QoS(1).transient_local());
  {
    std_msgs::msg::String serial_msg;
    serial_msg.data = serial_number_;
    serial_number_publisher_->publish(serial_msg);
    if (serial_number_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "serial_number is not set; publishing an empty string on "
                  "rover_%s/serial_number. Set it in the robot config.",
                  robot_type_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial number: %s", serial_number_.c_str());
    }
  }
  odometry_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(4));

  odometry_timer_ =
        create_wall_timer(1s / odometry_frequency_, [=]() { update_odom(); });
  robot_status_timer_ = create_wall_timer(1s / robot_status_frequency_,
                                          [=]() { publish_robot_status(); });
  RCLCPP_INFO(
      get_logger(),
      "Publishing Robot status on %s at %.2Fhz",
      robot_status_topic_.c_str(), robot_status_frequency_);


  // Init Pid
  if (control_mode_name_ == "TRACTION_CONTROL") {
    control_mode_ = Control::TRACTION_CONTROL;
    RCLCPP_INFO(get_logger(),
                "Control Mode is in TRACTION CONTROL; Drive with CAUTION");
  } else if (control_mode_name_ == "INDEPENDENT_WHEEL" || control_mode_name_ == "closed_loop") {
    control_mode_ = Control::INDEPENDENT_WHEEL;
    RCLCPP_INFO(get_logger(), "Robot is in closed loop mode.");
    RCLCPP_INFO(get_logger(), "PID is at P:%.4f I:%.4f D:%.4f", pi_p_, pi_i_, pi_d_);
  } else {
    control_mode_ = Control::OPEN_LOOP;
    RCLCPP_INFO(get_logger(), "Closed Loop Control is Disabled and Control Mode is in OPEN LOOP");
  }
  pid_gains_ = {pi_p_, pi_i_, pi_d_};
  // initialize connection to robot
  RCLCPP_INFO(get_logger(), "Connecting to robot at %s", device_port_.c_str());
  if (robot_type_ == "pro") {
    try {
      robot_ = std::make_unique<ProProtocolObject>(
          device_port_.c_str(), comm_type_, control_mode_, pid_gains_);
    } catch (int i) {
      RCLCPP_FATAL(get_logger(), "Error when connecting to robot.");
      if (i == SOCKET_CREATION_ERROR) {
        RCLCPP_FATAL(get_logger(), "Robot at %s is not available. Check that port is available and permissions allow access.", device_port_.c_str());
      } else if (i == -2) {
        RCLCPP_FATAL(get_logger(),
                     "This communication method is not supported on this robot. Please check the config files.");
      } else {
        RCLCPP_FATAL(get_logger(), "Unknown Error Occurred. Please try power cycling.");
      }
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to robot at %s", device_port_.c_str());
  } else if (robot_type_ == "zero2") {
    try {
      robot_ = std::make_unique<Zero2ProtocolObject>(
          device_port_.c_str(), comm_type_, control_mode_, pid_gains_,
          angular_scaling_params_);
    } catch (int i) {
      RCLCPP_FATAL(get_logger(), "Error when connecting to robot.");
      if (i == SOCKET_CREATION_ERROR) {
        RCLCPP_FATAL(get_logger(), "Robot at %s is not available. Check that port is available and permissions allow access.", device_port_.c_str());
      } else if (i == -2) {
        RCLCPP_FATAL(get_logger(),
                     "This communication method is not supported on this robot. Please check the config files.");
      } else {
        RCLCPP_FATAL(get_logger(), "Unknown Error Occurred. Please try power cycling.");
      }
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to robot at %s", device_port_.c_str());
  } else if (robot_type_ == "mini_2wd") {
    try {
      robot_ = std::make_unique<Mini2WDProtocolObject>(
          device_port_.c_str(),
          comm_type_,
          control_mode_,
          pid_gains_,
          angular_scaling_params_
      );
    } catch (int i) {
      RCLCPP_FATAL(get_logger(), "Error when connecting to Mini 2WD.");
      if (i == SOCKET_CREATION_ERROR) {
        RCLCPP_FATAL(get_logger(), "Device %s unavailable or permission denied.", device_port_.c_str());
      } else if (i == -2) {
        RCLCPP_FATAL(get_logger(), "mini_2wd requires comm_type=serial.");
      } else {
        RCLCPP_FATAL(get_logger(), "Unknown error.");
      }
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to Mini 2WD at %s", device_port_.c_str());
  } else if (robot_type_ == "mini" || robot_type_ == "miti" || robot_type_ == "max" || robot_type_ == "mega") {
    try {
      robot_ = std::make_unique<DifferentialRobot>(
          device_port_.c_str(), comm_type_, wheel_radius_, wheel_base_, robot_length_, pid_gains_, angular_scaling_params_, gear_ratio_, motor_pole_pairs_,
          control_mode_, rest_wheel_rpm_, brake_band_duty_, brake_band_rpm_,
          rpm_per_duty_, release_hold_s_);
    } catch (int i) {
      RCLCPP_FATAL(get_logger(), "Error when connecting to robot.");
      if (i == SOCKET_CREATION_ERROR) {
        RCLCPP_FATAL(get_logger(), "Robot at %s is not available. Check that port is available and permissions allow access.", device_port_.c_str());
      } else if (i == -2) {
        RCLCPP_FATAL(get_logger(),
                     "Error in the socket bind. Either could not find or access %s. Please check the device exists and has correct permissions.", device_port_.c_str());
      } else {
        RCLCPP_FATAL(get_logger(), "Unknown Error Occurred. Please try power cycling.");
      }
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to robot at %s", device_port_.c_str());
    RCLCPP_INFO(get_logger(), "Gear Ratio: %f", gear_ratio_);
    RCLCPP_INFO(get_logger(), "Motor pole pairs: %.0f", motor_pole_pairs_);
    RCLCPP_INFO(get_logger(), "rest_wheel_rpm %.1f control_mode %d",
                rest_wheel_rpm_, static_cast<int>(control_mode_));
    RCLCPP_INFO(get_logger(), "brake_band_duty %.3f brake_band_rpm %.1f rpm_per_duty %.1f release_hold_s %.2f",
                brake_band_duty_, brake_band_rpm_, rpm_per_duty_, release_hold_s_);
    if (brake_band_duty_ > 0.0f && control_mode_ == Control::TRACTION_CONTROL) {
      RCLCPP_WARN(get_logger(), "brake_band_duty is ignored in TRACTION_CONTROL");
    }
    if (control_mode_ == Control::OPEN_LOOP) {
      RCLCPP_WARN(get_logger(),
                  "OPEN_LOOP is not supported on %s; using INDEPENDENT_WHEEL",
                  robot_type_.c_str());
    }
    publish_joint_states_ = true;
    if (auto diff_robot = dynamic_cast<DifferentialRobot*>(robot_.get())) {
      diff_robot->set_wheel_trims(wheel_trim_fl_,
                                  wheel_trim_fr_,
                                  wheel_trim_rl_,
                                  wheel_trim_rr_);
      RCLCPP_INFO(get_logger(),
                  "Wheel trims set to FL=%.3f FR=%.3f RL=%.3f RR=%.3f",
                  wheel_trim_fl_, wheel_trim_fr_, wheel_trim_rl_, wheel_trim_rr_);
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "Robot Type is currently not suppported. Stopping this Node");
    rclcpp::shutdown();
  }

  /* start in the configured estop state; a reset on estop_reset_topic releases it */
  if (robot_) robot_->send_estop(estop_state_);
  publish_estop_status();

  velocity_timer_ = create_wall_timer(
        50ms,
        std::bind(&RobotDriver::publish_ramped_velocity, this)
      );

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RobotDriver::watchdog_tick, this));
}

void RobotDriver::publish_robot_info() {
  // RCLCPP_INFO(get_logger(), "Updating Robot Info");
  if (!robot_->is_connected()) {
    RCLCPP_FATAL(
        get_logger(),
        "Did not receive any data from the robot or the data is stale. Check that the robot is connected to the computer and that permissions are set correctly.");
    rclcpp::shutdown();
  }
  robot_data_ = robot_->info_request();
  std_msgs::msg::Float32MultiArray robot_info;
  robot_info.data.clear();
  robot_info.data.push_back(robot_data_.robot_guid);
  robot_info.data.push_back(robot_data_.robot_firmware);
  robot_info.data.push_back(robot_data_.robot_speed_limit);
  robot_info.data.push_back(robot_data_.robot_fan_speed);
  robot_info.data.push_back(robot_data_.robot_fault_flag);

  robot_info_publisher->publish(robot_info);
}

void RobotDriver::publish_robot_status() {
  // std::cerr << robot_->is_connected() << std::endl;
  if (!robot_->is_connected()) {
    RCLCPP_FATAL(
        get_logger(),
        "Did not receive any data from the robot or the data is stale. Check that the robot is connected to the computer and that permissions are set correctly.");
    rclcpp::shutdown();
  }
  // RCLCPP_INFO(get_logger(), "Updating Robot Status");
  robot_data_ = robot_->status_request();
  std_msgs::msg::Float32MultiArray robot_status;
  robot_status.data.clear();
  // Motor Infos
  robot_status.data.push_back(robot_data_.motor1_id);
  robot_status.data.push_back(robot_data_.motor1_rpm);
  robot_status.data.push_back(robot_data_.motor1_current);
  robot_status.data.push_back(robot_data_.motor1_temp);
  robot_status.data.push_back(robot_data_.motor1_mos_temp);
  robot_status.data.push_back(robot_data_.motor2_id);
  robot_status.data.push_back(robot_data_.motor2_rpm);
  robot_status.data.push_back(robot_data_.motor2_current);
  robot_status.data.push_back(robot_data_.motor2_temp);
  robot_status.data.push_back(robot_data_.motor2_mos_temp);
  robot_status.data.push_back(robot_data_.motor3_id);
  robot_status.data.push_back(robot_data_.motor3_rpm);
  robot_status.data.push_back(robot_data_.motor3_current);
  robot_status.data.push_back(robot_data_.motor3_temp);
  robot_status.data.push_back(robot_data_.motor3_mos_temp);
  robot_status.data.push_back(robot_data_.motor4_id);
  robot_status.data.push_back(robot_data_.motor4_rpm);
  robot_status.data.push_back(robot_data_.motor4_current);
  robot_status.data.push_back(robot_data_.motor4_temp);
  robot_status.data.push_back(robot_data_.motor4_mos_temp);
  // Battery Infos
  robot_status.data.push_back(robot_data_.battery1_voltage);
  robot_status.data.push_back(robot_data_.battery2_voltage);
  robot_status.data.push_back(robot_data_.battery1_temp);
  robot_status.data.push_back(robot_data_.battery2_temp);
  robot_status.data.push_back(robot_data_.battery1_current);
  robot_status.data.push_back(robot_data_.battery2_current);
  robot_status.data.push_back(robot_data_.battery1_SOC);
  robot_status.data.push_back(robot_data_.battery2_SOC);
  robot_status.data.push_back(robot_data_.battery1_fault_flag);
  robot_status.data.push_back(robot_data_.battery2_fault_flag);

  // Flipper Infos
  robot_status.data.push_back(robot_data_.motor3_angle);
  robot_status.data.push_back(robot_data_.motor3_sensor1);
  robot_status.data.push_back(robot_data_.motor3_sensor2);
  robot_status_publisher_->publish(robot_status);


  // Battery Status Topic
  auto battery_msg = sensor_msgs::msg::BatteryState();
  battery_msg.header.stamp = get_clock()->now();
  /* we only get here with a live connection, so a battery is reporting */
  battery_msg.present = true;
  if (robot_type_ != "pro"){
    battery_msg.percentage = robot_data_.battery1_SOC;
    battery_msg.voltage = robot_data_.battery1_voltage;
    battery_msg.current = robot_data_.battery1_current;
  } else {
    battery_msg.percentage = mapValue(robot_data_.battery1_SOC, inMin, inMax, outMin, outMax);
    battery_msg.voltage = robot_data_.battery1_SOC/29.94; //pro firmware reports voltage max as 970 and min as 770, hence the no. is divided by 29.94 to get the value in the actual range
    battery_msg.current = robot_data_.battery2_current;
  }
  battery_soc_publisher_->publish(battery_msg);
}

void RobotDriver::update_odom() {
  if (!robot_->is_connected()) {
    RCLCPP_FATAL(
        get_logger(),
        "Did not receive any data from the robot or the data is stale. Check that the robot is connected to the computer and that permissions are set correctly.");
    rclcpp::shutdown();
  }
  robot_data_ = robot_->status_request();
  
  // RCLCPP_INFO(get_logger(), "Updating Robot Odom");
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped odom_trans;

  
  // odom pose stuff
  double now_time = 0;
  double dt = 0;
  double mean_linear = 0;
  double mean_angular = 0;
  tf2::Quaternion q_new;
  
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = odom_child_frame_id_;
  odom.header.stamp = get_clock()->now();
  
  // Set up odom->base_link transform
  odom_trans.header.stamp = get_clock()->now();
  odom_trans.header.frame_id = odom_frame_id_;
  odom_trans.child_frame_id = odom_child_frame_id_;
  
  // Calculate time
  rclcpp::Time ros_now_time = get_clock()->now();
  now_time = ros_now_time.seconds();
  
  const bool first_sample = (last_odom_time_ == 0.0);
  dt = now_time - last_odom_time_;
  last_odom_time_ = now_time;

  /* the accumulators smooth what we publish as the current twist */
  linear_accumulator_.accumulate(robot_data_.linear_vel);
  angular_accumulator_.accumulate(robot_data_.angular_vel);
  
  mean_linear = linear_accumulator_.getRollingMean();
  mean_angular = angular_accumulator_.getRollingMean();

  // Calculate position
  if (!first_sample)
  {
    /* integrate the instantaneous velocity, not the rolling mean: the mean
     * lags the robot by half the window and that lag becomes position error
     * on every acceleration and deceleration */
    pos_x_ += robot_data_.linear_vel * cos(theta_) * dt;
    pos_y_ += robot_data_.linear_vel * sin(theta_) * dt;
    theta_ += robot_data_.angular_vel * dt;
    /* keep theta in [-pi, pi] so it does not grow without bound */
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
  }

  q_new.setRPY(0, 0, theta_);
  tf2::convert(q_new, odom_trans.transform.rotation);
  tf2::convert(q_new, odom.pose.pose.orientation);
  
  
  odom_trans.transform.translation.x = pos_x_;
  odom_trans.transform.translation.y = pos_y_;
  odom_trans.transform.translation.z = 0.0;
  //odom_trans.transform.rotation = q_new;
  

  odom.pose.pose.position.x = pos_x_;
  odom.pose.pose.position.y = pos_y_;
  odom.pose.pose.position.z = 0.0;
    
  odom.twist.twist.linear.x = mean_linear;
  odom.twist.twist.angular.z = mean_angular;
  
  /* Covariance. Row-major 6x6 over (x, y, z, roll, pitch, yaw), so the
   * diagonal is 0, 7, 14, 21, 28, 35. Pose is dead reckoned and drifts, so it
   * must not be left at zero: a fusion node reads all-zero as "no uncertainty"
   * and will trust wheel odometry over every other sensor. */
  odom.pose.covariance[0] = pose_linear_covariance;   // x
  odom.pose.covariance[7] = pose_linear_covariance;   // y
  odom.pose.covariance[35] = pose_yaw_covariance;     // yaw
  odom.pose.covariance[14] = UNOBSERVED_COVARIANCE;   // z
  odom.pose.covariance[21] = UNOBSERVED_COVARIANCE;   // roll
  odom.pose.covariance[28] = UNOBSERVED_COVARIANCE;   // pitch

  odom.twist.covariance[0] = linear_covariance;       // vx
  /* vy is structurally zero on a differential drive, so it is known, not
   * uncertain. Giving it the forward-velocity covariance tells a fusion node
   * the robot might be sliding sideways as fast as it drives. */
  odom.twist.covariance[7] = 1e-9;                    // vy
  odom.twist.covariance[35] = yaw_covariance;         // vyaw
  odom.twist.covariance[14] = UNOBSERVED_COVARIANCE;  // vz
  odom.twist.covariance[21] = UNOBSERVED_COVARIANCE;  // vroll
  odom.twist.covariance[28] = UNOBSERVED_COVARIANCE;  // vpitch
  	
  
  if (publish_joint_states_ && !first_sample) {
    publish_joint_states(dt);
  }

  // Publish odometry and odom->base_link transform
  odometry_publisher_->publish(odom);
  
  if(pub_odom_tf_){
    odom_tf_pub->sendTransform(odom_trans);
  }
}

void RobotDriver::publish_joint_states(double dt) {
  /* VESC ids 1..4 are FL, FR, BL, BR; the URDF joints are fl/fr/rl/rr.
   * Units follow the sensor_msgs/JointState convention for a revolute joint:
   *   position -> radians, cumulative wheel angle since this node started
   *   velocity -> radians/second
   * Multiply either by wheel_radius to get metres / metres per second. */
  const double rpm[4] = {robot_data_.motor1_rpm, robot_data_.motor2_rpm,
                         robot_data_.motor3_rpm, robot_data_.motor4_rpm};

  sensor_msgs::msg::JointState js;
  js.header.stamp = get_clock()->now();
  js.name = {"fl_wheel_to_chassis", "fr_wheel_to_chassis",
             "rl_wheel_to_chassis", "rr_wheel_to_chassis"};
  js.position.resize(4);
  js.velocity.resize(4);
  for (int i = 0; i < 4; i++) {
    const double rads = rpm[i] * 2.0 * M_PI / 60.0;
    wheel_angle_[i] += rads * dt;
    js.position[i] = wheel_angle_[i];
    js.velocity[i] = rads;
  }
  joint_state_publisher_->publish(js);
}

void RobotDriver::velocity_event_callback(
    geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  if (!robot_->is_connected()) {
    RCLCPP_FATAL(
        get_logger(),
        "Did not receive any data from the robot or the data is stale. Check that the robot is connected to the computer and that permissions are set correctly.");
    rclcpp::shutdown();
  }

  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "non-finite cmd_vel ignored");
    return;
  }
  last_cmd_time_ = steady_clock_.now();
  halted_ = false;
  target_linear_velocity_  = msg->linear.x;
  last_incoming_angular_z_ = msg->angular.z;
}

void RobotDriver::publish_ramped_velocity()
{
  /* ramp linear down only; acceleration is left to the PID */
  if (target_linear_velocity_ < last_linear_velocity_) {
    last_linear_velocity_ =
      std::max(target_linear_velocity_,
               last_linear_velocity_ - max_velocity_step_);
  } else {
    last_linear_velocity_ = target_linear_velocity_;
  }
  double speeddata[3];
  speeddata[0] = last_linear_velocity_;
  speeddata[1] = last_incoming_angular_z_;
  speeddata[2] = 0.0;  /* pass msg->angular.y here for mecanum */

  robot_->set_robot_velocity(speeddata);
}

void RobotDriver::trim_event_callback(
    std_msgs::msg::Float32::ConstSharedPtr &msg) {
  RCLCPP_INFO(get_logger(), "Trim Event triggered");
  robot_->update_drivetrim(msg->data);
}

void RobotDriver::estop_trigger_event_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    RCLCPP_INFO(get_logger(), "Software Estop activated");
    estop_state_ = true;
    robot_->send_estop(estop_state_);
    publish_estop_status();
  }
}

double RobotDriver::declare_number_(const std::string &name, double default_value) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  auto value = declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return value.get<double>();
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    return static_cast<double>(value.get<int64_t>());
  RCLCPP_ERROR(get_logger(), "parameter %s must be a number, using %f", name.c_str(), default_value);
  return default_value;
}

void RobotDriver::publish_estop_status() {
  std_msgs::msg::Bool msg;
  msg.data = estop_state_;
  estop_status_publisher_->publish(msg);
}

void RobotDriver::estop_reset_event_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    RCLCPP_INFO(get_logger(), "Software Estop deactivated");
    estop_state_ = false;
    robot_->send_estop(estop_state_);
    publish_estop_status();
  }
}

void RobotDriver::robot_info_request_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    publish_robot_info();
  }
}

void RobotDriver::watchdog_tick() {
  const double dt = (steady_clock_.now() - last_cmd_time_).seconds();
  if (dt > cmd_vel_timeout_sec_) {
    /* zero the targets and let publish_ramped_velocity(), the single writer,
     * ramp down; calling set_robot_velocity() here would race the ramp timer */
    target_linear_velocity_  = 0.0;
    last_incoming_angular_z_ = 0.0;
    if (!halted_) {
      RCLCPP_WARN(get_logger(), "cmd_vel timeout (%.2fs > %.2fs). HALT.", dt, cmd_vel_timeout_sec_);
      halted_ = true;
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto rover_node = std::make_shared<RobotDriver>();
  executor.add_node(rover_node);

  executor.spin();
  return 0;
}
