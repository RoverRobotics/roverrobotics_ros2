#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include "protocol_pro.hpp"
#include "protocol_zero_2.hpp"
#include "differential_robot.hpp"
#include "global_error_constants.hpp"

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// #include <librover/status_data.hpp>
using namespace std::chrono_literals;

using duration = std::chrono::nanoseconds;
namespace RoverRobotics {
/// This node supervises a Connection node and translates between low-level
/// commands and high-level commands.
class RobotDriver : public rclcpp::Node {
 public:
  RobotDriver();

 private:
  // Default Values
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
  const std::string ROBOT_STATUS_TOPIC_DEFAULT_ = "/robot_status";
  const float ROBOT_STATUS_FREQUENCY_DEFAULT_ = 60.0;
  const std::string ROBOT_INFO_REQUEST_TOPIC_DEFAULT_ = "/robot_info/request";
  const std::string ROBOT_INFO_TOPIC_DEFAULT_ = "/robot_info";
  const std::string ROBOT_TYPE_DEFAULT_ = "NONE";
  const std::string DEVICE_PORT_DEFAULT_ = "NONE";
  const std::string COMM_TYPE_DEFAULT_ = "NONE";
  const std::string SPEED_TOPIC_DEFAULT_ = "/cmd_vel/managed";
  const std::string ESTOP_TRIGGER_TOPIC_DEFAULT_ = "/soft_estop/trigger";
  const std::string ESTOP_RESET_TOPIC_DEFAULT_ = "/soft_estop/reset";
  const std::string TRIM_TOPIC_DEFAULT_ = "/trim_event";
  const bool ESTOP_STATE_DEFAULT_ = false;
  const std::string CONTROL_MODE_DEFAULT_ = "INDEPENDENT_WHEEL";
  const float LINEAR_TOP_SPEED_DEFAULT_ = 2;
  const float ANGULAR_TOP_SPEED_DEFAULT_ = 2;
  const bool PUB_ODOM_TF_DEFAULT_ = false;
  const float PID_P_DEFAULT_ = 0;
  const float PID_I_DEFAULT_ = 0;
  const float PID_D_DEFAULT_ = 0;
  const float LIN_COVAR_DEFAULT = 0.05;
  const float YAW_COVAR_DEFAULT = 0.4;
  const float ROBOT_ODOM_FREQUENCY_DEFAULT_ = 30;
  Control::angular_scaling_params angular_scaling_params_ = {0, 0, 0, 0, 0};
  const float ANGULAR_SCALING_A_DEFAULT_ = 0;
  const float ANGULAR_SCALING_B_DEFAULT_ = 0;
  const float ANGULAR_SCALING_C_DEFAULT_ = 0;
  const float ANGULAR_SCALING_MIN_DEFAULT_ = 1;
  const float ANGULAR_SCALING_MAX_DEFAULT_ = 1;
  const float WHEEL_RADIUS_DEFAULT_ = 0.08255;
  const float WHEEL_BASE_DEFAULT_ = 0.28575;
  const float ROBOT_LENGTH_DEFAULT_ = 0.2159;
  // robot protocol pointer
  std::unique_ptr<BaseProtocolObject> robot_;
  // universal robot data structure
  robotData robot_data_ = {};
  Control::pid_gains pid_gains_ = {0, 0, 0};
  // ROS 2 PUB SUB (5,3)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      speed_command_subscriber_;  // listen to cmd_vel inputs
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      trim_event_subscriber_;  // listen to trim event
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      estop_trigger_subscriber_;  // listen to estop trigger inputs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      estop_reset_subscriber_;  // listen to estop reset inputs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      robot_info__request_subscriber_;  // listen to robot_info request

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      robot_info_publisher;  // publish robot_unique info
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      robot_status_publisher_;  // publish robot state
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odometry_publisher_;  // Odom Publisher
   rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_soc_publisher_;  // Battery Status Publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_pub; // Odom TF Broadcaster

  // Timepoint / Timer
  rclcpp::Time odom_prev_time_;
  rclcpp::TimerBase::SharedPtr odometry_timer_;
  rclcpp::TimerBase::SharedPtr robot_status_timer_;

  // configurable variables
  std::string speed_topic_;
  std::string estop_trigger_topic_;
  std::string estop_reset_topic_;
  std::string robot_status_topic_;
  float robot_status_frequency_;
  std::string robot_info_request_topic_;
  std::string robot_info_topic_;
  std::string robot_type_;
  std::string trim_topic_;
  std::string device_port_;
  std::string comm_type_;
  float wheel_radius_;
  float wheel_base_;
  float robot_length_;
  std::string odom_topic_;

  // odom
  double odometry_frequency_;
  bool pub_odom_tf_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;

  // others
  int motors_id_[4] = {1, 2, 3, 4};
  bool estop_state_;
  std::string control_mode_name_;
  Control::robot_motion_mode_t control_mode_;
  float linear_covariance;
  float yaw_covariance;
  double linear_top_speed_;
  double angular_top_speed_;

  /**
   * @brief Ros2 Velocity Callback
   *
   * @param msg Twist Msg containing linear x y z and angular x y z
   */
  void velocity_event_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Trim Topic Event Callback
   *
   * @param msg Float value of trim delta to update
   */
  void trim_event_callback(std_msgs::msg::Float32::ConstSharedPtr &msg);
  /**
   * @brief Estop Trigger Topic Event Callback
   *
   * @param msg Bool msg to turn on Estop only (True = Estop On; False DO
   * NOTHING)
   */
  void estop_trigger_event_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  /**
   * @brief Estop Reset Topic Event Callback
   *
   * @param msg Bool msg to turn off Estop Only (True = Estop Off; False DO
   * NOTHING)
   */
  void estop_reset_event_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  /**
   * @brief Robot Unique Info Request Topic Event Callback
   *
   * @param msg Bool msg if you want robot unique informations (True= send msg;
   * False Do nothing)
   */
  void robot_info_request_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  /**
   * @brief Publish robot status at an interval
   *
   */
  void publish_robot_status();
  /**
   * @brief Publish robot info when robot_info_request contains the correct msg
   *
   */
  void publish_robot_info();
  /**
   * @brief Publish odom at an interval
   *
   */
  void update_odom();
};
}  // namespace RoverRobotics
