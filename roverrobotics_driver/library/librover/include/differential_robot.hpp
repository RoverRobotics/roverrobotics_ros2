#pragma once
#include "protocol_base.hpp" 
#include "vesc.hpp"
#include "utilities.hpp"

namespace RoverRobotics {
class DifferentialRobot;

enum VESC_IDS{
  FRONT_LEFT = 1,
  FRONT_RIGHT = 2,
  BACK_LEFT = 3,
  BACK_RIGHT = 4
};

}

class RoverRobotics::DifferentialRobot
    : public RoverRobotics::BaseProtocolObject {
 public:
  DifferentialRobot(const char *device, 
                     float wheel_radius,
                     float wheel_base,
                     float robot_length,
                     Control::pid_gains pid);

  void send_estop(bool) override;
  /*
   * @brief Request Robot Status
   * @return structure of statusData
   */
  robotData status_request() override;
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
   * @brief Attempt to make connection to robot via device
   * @param device is the address of the device (ttyUSB0 , can0, ttyACM0, etc)
   */
  void register_comm_base(const char *device) override;

 private:
  /*
   * @brief Thread Driven function that will send commands to the robot at set
   * interval
   * @param sleeptime sleep time between each cycle
   * @param datalist list of data to request
   */
  void send_command(int sleeptime);
  /*
   * @brief Thread Driven function update the robot motors using pid
   * @param sleeptime sleep time between each cycle
   */
  void motors_control_loop(int sleeptime);

  /* metric units (meters) */
  Control::robot_geometry robot_geometry_;

  const int MOTOR_NEUTRAL_ = 0;

  /* max: 1.0, min: 0.0  */
  const float MOTOR_MAX_ = .97;
  const float MOTOR_MIN_ = .02;
  float geometric_decay_ = .98;
  float left_trim_ = 1;
  float right_trim_ = 1;

  /* derivative of acceleration */
  const float LINEAR_JERK_LIMIT_ = 5;

  /* empirically measured */
  const float OPEN_LOOP_MAX_RPM_ = 600;


  int robotmode_num_ = Control::INDEPENDENT_WHEEL;

  const double CONTROL_LOOP_TIMEOUT_MS_ = 400;

  std::unique_ptr<Control::SkidRobotMotionController> skid_control_;
  std::unique_ptr<CommBase> comm_base_;
  std::string comm_type_;

  std::thread write_to_robot_thread_;
  std::thread motor_speed_update_thread_;
  std::mutex robotstatus_mutex_;

  /* main data structure */
  robotData robotstatus_;

  double motors_speeds_[4];
  double trimvalue_ = 0;
  
  bool estop_;

  Control::robot_motion_mode_t robot_mode_;
  Control::pid_gains pid_;
  Control::angular_scaling_params angular_scaling_params_;
  vesc::BridgedVescArray vescArray_;

};
