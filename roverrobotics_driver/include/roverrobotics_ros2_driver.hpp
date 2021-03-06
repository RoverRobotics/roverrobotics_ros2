#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librover/protocol_pro.hpp>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
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
  void velocity_event_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void trim_event_callback(std_msgs::msg::Float32::ConstSharedPtr &msg);
  void estop_trigger_event_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  void estop_reset_event_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  void robot_info_request_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
  void publish_robot_status();
  void publish_robot_info();
  void update_odom();
  // robot protocol pointer
  std::unique_ptr<BaseProtocolObject> robot_;
  // universal robot data structure
  robotData robot_data_ = {};
  PidGains pid_gains_ = {0, 0, 0};
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
  std::string odom_topic_;

  // motors
  int motors_id_[4] = {1, 2, 3, 4};
  bool estop_state_;
  bool closed_loop_;
  double linear_top_speed_;
  double angular_top_speed_;

  // odom
  double odometry_frequency_;
  bool pub_tf_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;
};
}  // namespace RoverRobotics
