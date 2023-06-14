#pragma once

#include "protocol_base.hpp"

namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  ProProtocolObject(const char* device, std::string new_comm_type,
                    Control::robot_motion_mode_t robot_mode,
                    Control::pid_gains pid);
  /*
   * @brief Trim Robot Velocity
   * Modify robot velocity differential (between the left side/right side) with
   * the input parameter. Useful to compensate if the robot tends to drift
   * either left or right while commanded to drive straight.
   * @param double of velocity offset
   */
  void update_drivetrim(double) override;
  /*
   * @brief Handle Estop Event
   * Send an estop event to the robot
   * @param bool accept a estop state
   */
  void send_estop(bool) override;
  /*
   * @brief Request Robot Status
   * @return structure of statusData
   */
  robotData status_request() override;
  /*
   * @brief Request Robot Unique Infomation
   * @return structure of statusData
   */
  robotData info_request() override;
  /*
   * @brief Set Robot velocity
   * Set Robot velocity: IF robot_mode_ TRUE, this function will attempt a
   * speed PID loop which uses all the available sensor data (wheels, IMUs, etc)
   * from the robot to produce the commanded velocity as best as possible. IF
   * robot_mode_ FALSE, this function simply translates the commanded
   * velocities into motor duty cycles and there is no expectation that the
   * commanded velocities will be realized by the robot. In robot_mode_ FALSE
   * mode, motor power is roughly proportional to commanded velocity.
   * @param controllarray an double array of control in m/s
   */
  void set_robot_velocity(double* controllarray) override;
  /*
   * @brief Unpack bytes from the robot
   * This is meant to use as a callback function when there are bytes available
   * to process
   * @param std::vector<uin32_t> Bytes stream from the robot
   * @return structure of statusData
   */
  void unpack_comm_response(std::vector<uint8_t>) override;
  /*
   * @brief Check if Communication still exist
   * @return bool
   */
  bool is_connected() override;
  /*
   * @brief Cycle through robot supported modes
   * @return int of the current mode enum
   */
  int cycle_robot_mode() override;
  /*
   * @brief Attempt to make connection to robot via device
   * @param device is the address of the device (ttyUSB0 , can0, ttyACM0)
   */
  void register_comm_base(const char* device) override;

 private:
  /*
   * @brief Thread Driven function that will send commands to the robot at set
   * interval
   * @param sleeptime sleep time between each cycle
   * @param datalist list of data to request
   */
  void send_command(int sleeptime, std::vector<uint32_t> datalist);
  /*
   * @brief Thread Driven function update the robot motors using pid
   * @param sleeptime sleep time between each cycle
   */
  void motors_control_loop(int sleeptime);
  const float MOTOR_GEAR_RATIO_ = 1.0 / 192.0; // Gear ratio from motor to wheel shaft including motor poles for converting from ERPM -> RPM
  const float MOTOR_DIST_PER_ROT_ = 0.8179; // 0.8179 Meters per rotation of wheel (wheel diameter of 10.25 inches / 0.26035 m)
  const float MOTOR_RPM_TO_MPS_RATIO_ = 1 / (MOTOR_GEAR_RATIO_ * MOTOR_DIST_PER_ROT_ / 60.0); // Divided by 60 to convert from minutes to seconds. 
  const int MOTOR_NEUTRAL_ = 125;
  const int MOTOR_MAX_ = 250;
  const int MOTOR_MIN_ = 0;
  const int OVF_FIXED_FIRM_VER_ = 10009; //based on abbcc format
  const unsigned char startbyte_ = 253;
  const int requestbyte_ = 10;
  const int termios_baud_code_ = 4097;  // THIS = baudrate of 57600
  const int RECEIVE_MSG_LEN_ = 5;
  const double wheel2wheelDistance = 0.365; // center distance between the wheels
  const double odom_angular_coef_ = 1/wheel2wheelDistance;
  const double odom_traction_factor_ = 0.610; // Default for 2WD is 0.9877, 4WD is 0.610, flipper is 0.98
  const double CONTROL_LOOP_TIMEOUT_MS_ = 200;
  std::unique_ptr<CommBase> comm_base_;
  std::string comm_type_;

  std::mutex robotstatus_mutex_;
  robotData robotstatus_;
  double motors_speeds_[3];
  double trimvalue_;
  std::thread fast_data_write_thread_;
  std::thread slow_data_write_thread_;
  std::thread motor_commands_update_thread_;
  bool estop_;
  bool closed_loop_;
  // Motor PID variables
  OdomControl motor1_control_;
  OdomControl motor2_control_;
  Control::robot_motion_mode_t robot_mode_;
  Control::pid_gains pid_;

  enum robot_motors { LEFT_MOTOR, RIGHT_MOTOR, FLIPPER_MOTOR };

  enum uart_param {
    REG_PWR_TOTAL_CURRENT = 0,
    REG_MOTOR_FB_RPM_LEFT = 2,
    REG_MOTOR_FB_RPM_RIGHT = 4,
    REG_FLIPPER_FB_POSITION_POT1 = 6,
    REG_FLIPPER_FB_POSITION_POT2 = 8,
    REG_MOTOR_FB_CURRENT_LEFT = 10,
    REG_MOTOR_FB_CURRENT_RIGHT = 12,
    REG_MOTOR_ENCODER_COUNT_LEFT = 14,
    REG_MOTOR_ENCODER_COUNT_RIGHT = 16,
    REG_MOTOR_FAULT_FLAG_LEFT = 18,
    REG_MOTOR_TEMP_LEFT = 20,
    REG_MOTOR_TEMP_RIGHT = 22,
    REG_PWR_BAT_VOLTAGE_A = 24,
    REG_PWR_BAT_VOLTAGE_B = 26,
    EncoderInterval_0 = 28,
    EncoderInterval_1 = 30,
    EncoderInterval_2 = 32,
    REG_ROBOT_REL_SOC_A = 34,
    REG_ROBOT_REL_SOC_B = 36,
    REG_MOTOR_CHARGER_STATE = 38,
    BuildNO = 40,
    REG_PWR_A_CURRENT = 42,
    REG_PWR_B_CURRENT = 44,
    REG_MOTOR_FLIPPER_ANGLE = 46,
    to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48,
    to_computer_REG_MOTOR_SLOW_SPEED = 50,
    BATTERY_STATUS_A = 52,
    BATTERY_STATUS_B = 54,
    BATTERY_MODE_A = 56,
    BATTERY_MODE_B = 58,
    BATTERY_TEMP_A = 60,
    BATTERY_TEMP_B = 62,
    BATTERY_VOLTAGE_A = 64,
    BATTERY_VOLTAGE_B = 66,
    BATTERY_CURRENT_A = 68,
    BATTERY_CURRENT_B = 70
  };
};
