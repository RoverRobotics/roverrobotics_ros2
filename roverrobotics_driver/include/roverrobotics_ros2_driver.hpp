#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include "protocol_pro.hpp"
#include "protocol_zero_2.hpp"
#include "protocol_mini_2wd_serial.hpp"
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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
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
  const float GEAR_RATIO_DEFAULT_ = 1;
  const float MOTOR_POLE_PAIRS_DEFAULT_ = 15;
  const std::string SERIAL_NUMBER_DEFAULT_ = "";
  const float MAX_VELOCITY_STEP_DEFAULT_ = 0.75;
  const float CMD_VEL_TIMEOUT_DEFAULT_ = 0.3f;  // 300 ms
  const double REST_WHEEL_RPM_DEFAULT_ = 8.0;
  const double BRAKE_BAND_DUTY_DEFAULT_ = 0.0;
  const double BRAKE_BAND_RPM_DEFAULT_ = 0.0;
  const double RPM_PER_DUTY_DEFAULT_ = 330.0;
  const double RPM_PER_DUTY_MIN_ = 300.0;  // with the band on, outside this range = typo, band off
  const double RPM_PER_DUTY_MAX_ = 340.0;
  const double RELEASE_HOLD_S_DEFAULT_ = 0.0;
  const float WHEEL_TRIM_FL_DEFAULT_ = 1.0f;
  const float WHEEL_TRIM_FR_DEFAULT_ = 1.0f;
  const float WHEEL_TRIM_RL_DEFAULT_ = 1.0f;
  const float WHEEL_TRIM_RR_DEFAULT_ = 1.0f;
  const float PID_P_DEFAULT_ = 0;
  const float PID_I_DEFAULT_ = 0;
  const float PID_D_DEFAULT_ = 0;
  const float LIN_COVAR_DEFAULT = 0.05;
  const float YAW_COVAR_DEFAULT = 0.4;
  const float POSE_LIN_COVAR_DEFAULT = 0.1;
  const float POSE_YAW_COVAR_DEFAULT = 0.5;
  /* z, roll and pitch are not observable on a planar drive; a large value
   * tells a fusion node to ignore them rather than trust a zero */
  const double UNOBSERVED_COVARIANCE = 1e6;
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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;  // per-wheel position and velocity
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      serial_number_publisher_;  // latched, published once at startup
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
  float gear_ratio_;
  float motor_pole_pairs_;
  std::string serial_number_;

  /* per-wheel joint state. Only the four-driven-wheel robots have the
   * fl/fr/rl/rr_wheel_to_chassis joints, so this stays off elsewhere. */
  /* integrated pose. Members rather than function statics so they can be
   * reset at runtime and are not shared between composed instances. */
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double theta_ = 0.0;
  double last_odom_time_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_odometry_subscriber_;

  bool publish_joint_states_ = false;
  double wheel_angle_[4] = {0.0, 0.0, 0.0, 0.0};
  void publish_joint_states(double dt);
  float wheel_trim_fl_;
  float wheel_trim_fr_;
  float wheel_trim_rl_;
  float wheel_trim_rr_;
  float max_velocity_step_;
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
  float pose_linear_covariance;
  float pose_yaw_covariance;
  double linear_top_speed_;
  double angular_top_speed_;

  double target_linear_velocity_  = 0.0;
  double last_linear_velocity_ = 0.0;
  double last_incoming_angular_z_ = 0.0;

  double cmd_vel_timeout_sec_;
  /* steady clock so a wall-clock jump (NTP at boot) can't defeat the timeout */
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_cmd_time_;
  float rest_wheel_rpm_;
  float brake_band_duty_;
  float brake_band_rpm_;
  float rpm_per_duty_;
  float release_hold_s_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::TimerBase::SharedPtr velocity_timer_;

  void watchdog_tick();

  void publish_ramped_velocity();

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
