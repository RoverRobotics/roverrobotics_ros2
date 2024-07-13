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
  robot_status_frequency_ = declare_parameter("robot_status_frequency",
                                              ROBOT_STATUS_FREQUENCY_DEFAULT_);
  robot_info_request_topic_ = declare_parameter(
      "robot_info_request_topic", ROBOT_INFO_REQUEST_TOPIC_DEFAULT_);
  robot_info_topic_ =
      declare_parameter("robot_info_topic", ROBOT_INFO_TOPIC_DEFAULT_);
  robot_type_ = declare_parameter("robot_type", ROBOT_TYPE_DEFAULT_);
  device_port_ = declare_parameter("device_port", DEVICE_PORT_DEFAULT_);
  comm_type_ = declare_parameter("comm_type", COMM_TYPE_DEFAULT_);
  wheel_radius_ = declare_parameter("wheel_radius", WHEEL_RADIUS_DEFAULT_);
  wheel_base_ = declare_parameter("wheel_base", WHEEL_BASE_DEFAULT_);
  robot_length_ = declare_parameter("robot_length", ROBOT_LENGTH_DEFAULT_);
  // Drive
  speed_topic_ = declare_parameter("speed_topic", SPEED_TOPIC_DEFAULT_);
  estop_trigger_topic_ =
      declare_parameter("estop_trigger_topic", ESTOP_TRIGGER_TOPIC_DEFAULT_);
  estop_reset_topic_ =
      declare_parameter("estop_reset_topic", ESTOP_RESET_TOPIC_DEFAULT_);
  trim_topic_ = declare_parameter("trim_topic", TRIM_TOPIC_DEFAULT_);
  estop_state_ = declare_parameter("estop_state", ESTOP_STATE_DEFAULT_);
  control_mode_name_ = declare_parameter("control_mode", CONTROL_MODE_DEFAULT_);
  linear_top_speed_ =
      declare_parameter("linear_top_speed", LINEAR_TOP_SPEED_DEFAULT_);
  angular_top_speed_ =
      declare_parameter("angular_top_speed", ANGULAR_TOP_SPEED_DEFAULT_);
  float pi_p_ = declare_parameter("motor_control_p_gain", PID_P_DEFAULT_);
  float pi_i_ = declare_parameter("motor_control_i_gain", PID_I_DEFAULT_);
  float pi_d_ = declare_parameter("motor_control_d_gain", PID_D_DEFAULT_);
  linear_covariance = declare_parameter("linear_covariance", LIN_COVAR_DEFAULT);
  yaw_covariance = declare_parameter("yaw_covariance", YAW_COVAR_DEFAULT);
  
  linear_accumulator_ = RollingMeanAccumulator(10);
  angular_accumulator_ = RollingMeanAccumulator(10);
  // Odom
  pub_odom_tf_ = declare_parameter("publish_tf", PUB_ODOM_TF_DEFAULT_);
  odom_topic_ = declare_parameter("odom_topic", "/odom_raw");
  odometry_frequency_ =
      declare_parameter("odometry_frequency", ROBOT_ODOM_FREQUENCY_DEFAULT_);
  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  odom_child_frame_id_ =
      declare_parameter("odom_child_frame_id", "base_link");
  // Angular Scaling params
  angular_scaling_params_.a_coef =
      declare_parameter("angular_a_coef", ANGULAR_SCALING_A_DEFAULT_);
  angular_scaling_params_.b_coef =
      declare_parameter("angular_b_coef", ANGULAR_SCALING_B_DEFAULT_);
  angular_scaling_params_.c_coef =
      declare_parameter("angular_c_coef", ANGULAR_SCALING_C_DEFAULT_);
  angular_scaling_params_.min_scale_val =
      declare_parameter("angular_min_scale", ANGULAR_SCALING_MIN_DEFAULT_);
  angular_scaling_params_.max_scale_val =
      declare_parameter("angular_max_scale", ANGULAR_SCALING_MAX_DEFAULT_);
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
  robot_info__request_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      estop_reset_topic_, rclcpp::QoS(2),
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
  } else if (robot_type_ == "mini" || robot_type_ == "miti" || robot_type_ == "max") {
    try {
      robot_ = std::make_unique<DifferentialRobot>(
          device_port_.c_str(), comm_type_, wheel_radius_, wheel_base_, robot_length_, pid_gains_, angular_scaling_params_);
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
  } else {
    RCLCPP_WARN(get_logger(),
                "Robot Type is currently not suppported. Stopping this Node");
    rclcpp::shutdown();
  }
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
  if (robot_type_ != "pro"){
    battery_msg.percentage = robot_data_.battery1_SOC;
    battery_msg.voltage = robot_data_.battery1_voltage;
    battery_msg.current = robot_data_.battery1_current;
  } else {
    battery_msg.percentage = mapValue(robot_data_.battery1_SOC, inMin, inMax, outMin, outMax);
    battery_msg.voltage = robot_data_.battery1_SOC/29.94; //pro firmware reports voltage max as 970 and min as 770, hence the nu. is divided by 29.94 to get the value in the actual range 
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
  static double pos_x = 0;
  static double pos_y = 0;
  static double theta = 0;
  static double past_time = 0;
  static double now_time = 0;
  static double dt = 0;
  static double mean_linear = 0;
  static double mean_angular = 0;
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
  
  dt = now_time - past_time;
  past_time = now_time;
  
  linear_accumulator_.accumulate(robot_data_.linear_vel);
  angular_accumulator_.accumulate(robot_data_.angular_vel);
  
  mean_linear = linear_accumulator_.getRollingMean();
  mean_angular = angular_accumulator_.getRollingMean();

  // Calculate position
  if (past_time != 0)
  {
    pos_x = pos_x + mean_linear * cos(theta) * dt;
    pos_y = pos_y + mean_linear * sin(theta) * dt;
    theta = (theta + mean_angular * dt);
    
    q_new.setRPY(0, 0, theta);
    tf2::convert(q_new, odom_trans.transform.rotation);
    tf2::convert(q_new, odom.pose.pose.orientation);
  }
  
  
  odom_trans.transform.translation.x = pos_x;
  odom_trans.transform.translation.y = pos_y;
  odom_trans.transform.translation.z = 0.0;
  //odom_trans.transform.rotation = q_new;
  

  odom.pose.pose.position.x = pos_x;
  odom.pose.pose.position.y = pos_y;
  odom.pose.pose.position.z = 0.0;
    
  odom.twist.twist.linear.x = mean_linear;
  odom.twist.twist.angular.z = mean_angular;
  
  // Covariance: 
  // If not moving, trust the encoders completely
  // Otherwise set them to the ROS param
  
  
  odom.twist.covariance[0] = linear_covariance;
  odom.twist.covariance[7] = linear_covariance;
  odom.twist.covariance[35] = yaw_covariance;
  	
  
  // Publish odometry and odom->base_link transform
  odometry_publisher_->publish(odom);
  
  if(pub_odom_tf_){
    odom_tf_pub->sendTransform(odom_trans);
  }
}

void RobotDriver::velocity_event_callback(
    geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  if (!robot_->is_connected()) {
    RCLCPP_FATAL(
        get_logger(),
        "Did not receive any data from the robot or the data is stale. Check that the robot is connected to the computer and that permissions are set correctly.");
    rclcpp::shutdown();
  }
  static double speeddata[3];
  speeddata[0] = msg->linear.x;
  speeddata[1] = msg->angular.z;
  speeddata[2] = msg->angular.y;
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
  }
}

void RobotDriver::estop_reset_event_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    RCLCPP_INFO(get_logger(), "Software Estop deactivated");
    estop_state_ = false;
    robot_->send_estop(estop_state_);
  }
}

void RobotDriver::robot_info_request_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    publish_robot_info();
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
