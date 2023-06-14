// Use to create utility classes that share across all robot.
#pragma once
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace RoverRobotics {
struct PidGains {
  double Kp;
  double Ki;
  double Kd;
  PidGains();
  PidGains(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}
};
class OdomControl {
 public:
  /*
   * @brief Odom Control Default contructor
   */
  OdomControl();
  /*
   * @brief Odom Control contructor
   * @param use_control A boolean to enable or disable odom control functions
   * @param pid_gains A structure of (double P , Double I, Double D) gains.
   * @param max A double to store max_velocity for controlled motor
   * @param min A double to store min_velocity for controlled motor
   */
  OdomControl(bool use_control, PidGains pid_gains, double max,
              double min);  // max min values for returned value
  /*
   * @brief Odom Control contructor with debug stream
   * @param use_control A boolean to enable or disable odom control functions
   * @param pid_gains A structure of (double P , Double I, Double D) gains.
   * @param max A double to store max_velocity for controlled motor
   * @param min A double to store min_velocity for controlled motor
   */
  OdomControl(bool use_control, PidGains pid_gains, double max, double min,
              std::ofstream* fs);  // max min values for returned value
  /*
   * @brief use commaned velocity and compare with measured_vel and applied Pids
   * @param commanded_vel A double
   * @param measured_vel A structure of (double P , Double I, Double D) gains.
   * @param dt A double to store max_velocity for controlled motor
   * @param firmwareBuildNumber A double to store min_velocity for controlled
   * motor
   * @return
   */
  double run(double commanded_vel, double measured_vel, double dt,
             int firmwareBuildNumber);  // in m/s
  void reset();
  double boundMotorSpeed(double motor_speed, double max, double min);
  
  double MOTOR_MAX_VEL_;
  double MOTOR_MIN_VEL_;
  double MOTOR_DEADBAND_;
  double MAX_ACCEL_CUTOFF_;  // 20

  bool use_control_;

  // Can poll these values to see if motor speed is saturating
  bool at_max_motor_speed_;
  bool at_min_motor_speed_;
  bool stop_integrating_;

  //.csv Debuggin
  std::ofstream* fs_;

  // General Class variables
  double K_P_;
  double K_I_;
  double K_D_;
  double integral_error_;
  double differential_error_;
  double velocity_error_;

  // Returned value
  double motor_command_vel_;
  double deadband_offset_;

  // velocity feedback
  double velocity_commanded_;
  double velocity_measured_;
  double measured_vel_;
  std::vector<double> velocity_filtered_history_;
  std::vector<double> velocity_history_;
  bool velocity_control_on_;
  bool skip_measurement_;

 private:
  bool hasZeroHistory(const std::vector<double>& vel_history);
  double deadbandOffset(double motor_speed, double deadband_offset);
  double P(double error);
  double I(double error, double dt);
  double D(double error, double dt);
  double PID(double error, double dt);
  double feedThroughControl();
};
}  // namespace RoverRobotics