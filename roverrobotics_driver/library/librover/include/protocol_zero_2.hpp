#pragma once

#include "protocol_base.hpp"
#include "utilities.hpp"
namespace RoverRobotics
{
  class Zero2ProtocolObject;
}
class RoverRobotics::Zero2ProtocolObject
    : public RoverRobotics::BaseProtocolObject
{
private:
  std::unique_ptr<Utilities::PersistentParams> persistent_params_;
  const std::string ROBOT_PARAM_PATH = strcat(std::getenv("HOME"), "/robot.config");
  Control::robot_geometry robot_geometry_ = {.intra_axle_distance = 0.2794,
                                             .wheel_base = 0.3683,
                                             .wheel_radius = 0.2667,
                                             .center_of_mass_x_offset = 0,
                                             .center_of_mass_y_offset = 0};
  const float MOTOR_RPM_TO_WHEEL_RPM_RATIO_ = 96 *2; 
  const float OPEN_LOOP_MAX_RPM_ = 17000 / MOTOR_RPM_TO_WHEEL_RPM_RATIO_;
  /* limit to the trim that can be applied; more than this means a robot issue*/
  const float MAX_CURVATURE_CORRECTION_ = .15;
  const int MOTOR_NEUTRAL_ = 0;
  const float MOTOR_MAX_ = 0.95;
  const float MOTOR_MIN_ = -0.95;
  const float LINEAR_JERK_LIMIT_ = 50;
  const double odom_angular_coef_ = 2.3;    
  const double odom_traction_factor_ = 0.7; 
  const double CONTROL_LOOP_TIMEOUT_MS_ = 200;
  const uint8_t PAYLOAD_BYTE_SIZE_ = 2;
  const uint8_t STOP_BYTE_ = 3;
  const uint8_t MSG_SIZE_ = 5;
  const uint8_t FORWARD_MSG_SIZE_ = 7;
  const uint8_t START_BYTE_ = 2;
  const int termios_baud_code_ = 4098; // THIS = baudrate of 115200
  const int RECEIVE_MSG_LEN_ = 1;
  float left_trim_ = 1;
  float right_trim_ = 1;
  float geometric_decay_ = .99;
  int robotmode_num_ = 0;
  const int ROBOT_MODES_ = 2;
  std::unique_ptr<Control::SkidRobotMotionController> skid_control_;
  std::unique_ptr<CommBase> comm_base_;
  std::string comm_type_;

  std::mutex robotstatus_mutex_;
  robotData robotstatus_;
  double motors_speeds_[2];
  double trimvalue_;
  std::thread write_to_robot_thread_;
  std::thread slow_data_write_thread_;
  std::thread motor_speed_update_thread_;
  bool estop_;
  bool closed_loop_;
  // Motor PID variables
  OdomControl motor1_control_;
  OdomControl motor2_control_;
  Control::robot_motion_mode_t robot_mode_;
  Control::angular_scaling_params angular_scaling_params_;
  Control::pid_gains pid_;
  double vesc_fet_temp_;
  double vesc_motor_temp_;
  float vesc_all_motor_current_;
  float vesc_all_input_current_;
  double vesc_id_;
  double vesc_iq_;
  float vesc_duty_;
  int vesc_rpm_;
  double vesc_v_in_;
  double vesc_amp_hours_;
  double vesc_amp_hours_charged_;
  double vesc_watt_hours_;
  double vesc_watt_hours_charged_;
  int vesc_tach_;
  int vesc_tach_abs_;
  int vesc_fault_;
  double vesc_speed_;
  uint vesc_dev_id_;
  double vesc_pid_pos_;

  enum robot_motors
  {
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 8
  };
  /*
   * @brief Thread Driven function that will send commands to the robot at set
   * interval to get its data
   * @param sleeptime sleep time between each cycle
   */
  void send_getvalues_command(int sleeptime);
  /*
   * @brief Helper function that will send motors commands to the robot at set
   * interval of the motor control loops thread
   */
  void send_motors_commands();
  /*
   * @brief Thread Driven function update the robot motors using pid
   * @param sleeptime sleep time between each cycle
   */
  void motors_control_loop(int sleeptime);

  /*
   * @brief loads the persistent parameters from a non-volatile config file
   * 
   */
  void load_persistent_params();

  const unsigned short crc16_tab[256] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                                         0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                                         0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                                         0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                                         0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                                         0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                                         0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                                         0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                                         0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                                         0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                                         0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                                         0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                                         0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                                         0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                                         0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                                         0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                                         0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                                         0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                                         0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                                         0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                                         0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                                         0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                                         0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                                         0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                                         0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                                         0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                                         0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                                         0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                                         0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

  unsigned short crc16(unsigned char *buf, unsigned int len);

public:
  Zero2ProtocolObject(const char *device, std::string new_comm_type,
                      Control::robot_motion_mode_t robot_mode,
                      Control::pid_gains pid, Control::angular_scaling_params angular_scale);
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
  void set_robot_velocity(double *controllarray) override;
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
  void register_comm_base(const char *device) override;

  enum uart_param
  {
    COMM_GET_VALUES = 4,
    COMM_SET_DUTY = 5,
    COMM_CAN_FORWARD = 34
  };
};
