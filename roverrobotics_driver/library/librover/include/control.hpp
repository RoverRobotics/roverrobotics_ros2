#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef DEBUG
#include <ctime>
#include <sstream>
#endif

#define RPM_TO_RADS_SEC 0.10472

namespace Control {

/* classes */
class PidController;
class SkidRobotMotionController;
class AlphaBetaFilter;

/* datatypes */
typedef enum {
  OPEN_LOOP = 0,
  TRACTION_CONTROL = 1,
  INDEPENDENT_WHEEL = 2,
} robot_motion_mode_t;

const uint8_t NUM_MOTION_MODES = 3;

struct angular_scaling_params {
  float a_coef;
  float b_coef;
  float c_coef;
  float min_scale_val;
  float max_scale_val;
};

struct robot_velocities {
  float linear_velocity;
  float angular_velocity;
};

struct motor_data {
  float fl;
  float fr;
  float rl;
  float rr;
};

struct robot_geometry {
  float intra_axle_distance;
  float wheel_base;
  float wheel_radius;
  float center_of_mass_x_offset;
  float center_of_mass_y_offset;
};

struct pid_gains {
  double kp;
  double ki;
  double kd;
};

struct pid_outputs {
  std::string name;
  double time;
  float dt;
  float pid_output;
  float error;
  float integral_error;
  float delta_error;
  float target_value;
  float measured_value;
  double kp;
  double ki;
  double kd;
};

struct pid_output_limits {
  float posmax;
  float negmax;
};

/* useful functions */

/*
 * @brief Translate linear and angular commands into target wheelspeeds based on
 * robot geometery
 * @param target_velocities is the target linear and angular velocities
 * @param robot_geometry is a description of the robot geometry
 */
motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry);

/*
 * @brief Limit the acceleration and deceleration of the robot (prevent
 * tipping/jerking)
 * @param target_velocities is the target linear and angular velocities
 * @param measured_velocities is the robot's measured linear and angular
 * velocities
 * @param delta_v_limits is the acceleration limits for linear and angular
 * velocities
 * @param dt is the time delta between this call to limitAcceleration() and the
 * previous call
 */
robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt);

/*
 * @brief Applies scaling to the angular command based on the current linear
 * velocity
 * @param target_velocities is the target linear and angular velocities
 * @param measured_velocities is the robot's measured linear and angular
 * velocities
 * @param scaling_params is 2nd order polynomial coefficients for scaling. The
 * output is: final_angular_command = [ax**2 + bx + c] * input_angular_command ,
 * where x is the current linear velocity
 */
robot_velocities scaleAngularCommand(robot_velocities target_velocities,
                                     robot_velocities measured_velocities,
                                     angular_scaling_params scaling_params);

/*
 * @brief Computes estimated robot velocities (linear, angular) from wheelspeeds
 * (ie rpms) and robot geometry
 * @param wheel_speeds rpm data for each wheel
 * @param robot_geometry is a description of the robot's geometry
 */
robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry);

}  // namespace Control

class Control::PidController {
 public:
  /* constructors */

  /*
   * @brief A class for generic PID control
   * @param pid_gains P, I, and D terms/coefficients/gains
   * @param name is a human readable name which is assigned to this instance of
   * the PID controller
   */
  PidController(pid_gains pid_gains, std::string name);

  /*
   * @brief A class for generic PID control
   * @param pid_gains P, I, and D terms/coefficients/gains
   * @param name is a human readable name which is assigned to this instance of
   * the PID controller
   * @param pid_output_limits sets output limits for systems which have bound
   * input ranges (ie a valve which operates on the control range of [0, 1])
   */
  PidController(pid_gains pid_gains, pid_output_limits pid_output_limits,
                std::string name);

  /*
   * @brief set the pid gains (P, I, D)
   * @param pid_gains P, I, and D terms/coefficients/gains
   */
  void setGains(pid_gains pid_gains);

  /*
   * @brief get the pid gains (P, I, D)
   */
  pid_gains getGains();

  /*
   * @brief set the output limits of the PID output
   * @param pid_output_limits sets output limits for systems which have bound
   * input ranges (ie a valve which operates on the control range of [0, 1])
   */
  void setOutputLimits(pid_output_limits pid_output_limits);

  /*
   * @brief get the output limits of the PID output
   */
  pid_output_limits getOutputLimits();

  /*
   * @brief limit the amount of error that can be accumulated/integrated for the
   * I-term
   * @param error_limit is the limit to the error that can be accumulated
   */
  void setIntegralErrorLimit(float error_limit);

  /*
   * @brief limit the amount of error that can be accumulated/integrated for the
   * I-term
   */
  float getIntegralErrorLimit();

  /*
   * @brief run the PID loop, compute the PID control output
   * @param target is the desired value
   * @param measured is the measured value of the system at present
   */
  pid_outputs runControl(float target, float measured);

  /*
   * @brief a datalogging function for the PID class
   * @param log_file is a file handle for the output file
   * @param data is the pid_output data
   */
  void writePidDataToCsv(std::ofstream &log_file, pid_outputs data);

 private:
  std::string name_;
  double kp_;
  double ki_;
  double kd_;
  float integral_error_;
  float integral_error_limit_;
  float previous_error_;
  float pos_max_output_;
  float neg_max_output_;
  std::chrono::steady_clock::time_point time_last_;
  std::chrono::steady_clock::time_point time_origin_;
};

class Control::SkidRobotMotionController {
 public:
  /* constructors */

  /*
   * @brief a class to compute motor commands based on desired velocities, robot
   * geometry, and calibration values
   */
  SkidRobotMotionController();

  /*
   * @brief a class to compute motor commands based on desired velocities, robot
   * geometry, and calibration values
   * @param operating_mode which type of control logic is used to compute wheel
   * commands
   * @param robot_geometry a description of the robot's geometry
   * @param max_motor_duty a hard-limit to the max duty cycle output
   * @param min_motor_duty a hard-limit to the min duty cycle output (anything
   * below this will be zeroed out)
   * @param right_trim a simple multiplier on the range [0, 1] which can scale
   * down the right-side power (useful if the robot curves left naturally)
   * @param left_trim a simple multiplier on the range [0, 1] which can scale
   * down the left-side power (useful if the robot curves right naturally)
   * @param open_loop_max_motor_rpm a description of how fast the wheels spin at
   * full power with no load
   */
  SkidRobotMotionController(robot_motion_mode_t operating_mode,
                            robot_geometry robot_geometry,
                            float max_motor_duty = 0.95,
                            float min_motor_duty = 0.03, float left_trim = 1.0,
                            float right_trim = 1.0,
                            float open_loop_max_motor_rpm = 600);

  /*
   * @brief a class to compute motor commands based on desired velocities, robot
   * geometry, and calibration values
   * @param operating_mode which type of control logic is used to compute wheel
   * commands
   * @param robot_geometry a description of the robot's geometry
   * @param max_motor_duty a hard-limit to the max duty cycle output
   * @param min_motor_duty a hard-limit to the min duty cycle output (anything
   * below this will be zeroed out)
   * @param right_trim a simple multiplier on the range [0, 1] which can scale
   * down the right-side power (useful if the robot curves left naturally)
   * @param left_trim a simple multiplier on the range [0, 1] which can scale
   * down the left-side power (useful if the robot curves right naturally)
   * @param geometric_decay applies a decay to the robot's duty cycle to ensure
   * it converges to 0 when the robot is inactive for a long-period of time
   */
  SkidRobotMotionController(robot_motion_mode_t operating_mode,
                            robot_geometry robot_geometry, pid_gains pid_gains,
                            float max_motor_duty = 0.95,
                            float min_motor_duty = 0.03, float left_trim = 1.0,
                            float right_trim = 1.0,
                            float geometric_decay = 0.99);

  /*
   * @brief set limits to robot acceleration/decceleration
   * @param limits acceleration limits
   */
  void setAccelerationLimits(robot_velocities limits);

  /*
   * @brief get limits to robot acceleration/decceleration
   */
  robot_velocities getAccelerationLimits();

  /*
   * @brief set robot operating mode (aka Drive control type)
   * @param operating_mode is the desired control mode
   */
  void setOperatingMode(robot_motion_mode_t operating_mode);

  /*
   * @brief get robot operating mode (aka Drive control type)
   */
  robot_motion_mode_t getOperatingMode();

  /*
   * @brief set the robot geometry parameters which are used in odometry and
   * control
   * @param robot_geometry is a description of the robot's geometry
   */
  void setRobotGeometry(robot_geometry robot_geometry);

  /*
   * @brief get the robot geometry parameters which are used in odometry and
   * control
   */
  robot_geometry getRobotGeometry();

  /*
   * @brief set the pid gains used in closed-loop control on the wheelspeeds
   * @param pid_gains is the P, I, and D gains
   */
  void setPidGains(pid_gains pid_gains);

  /*
   * @brief get the pid gains used in closed-loop control on the wheelspeeds
   */
  pid_gains getPidGains();

  /*
   * @brief set the max allowable motor duty cycle, not to be exceeded by
   * control loops
   * @param max_motor_duty is the desired maximum duty limit
   */
  void setMotorMaxDuty(float max_motor_duty);

  /*
   * @brief get the max allowable motor duty cycle, not to be exceeded by
   * control loops
   */
  float getMotorMaxDuty();

  /*
   * @brief set the min allowable motor duty cycle
   * @param max_min_duty is the minimum allowable duty, anything control output
   * below this will be truncated to 0.
   */
  void setMotorMinDuty(float max_min_duty);

  /*
   * @brief get the min allowable motor duty cycle
   */
  float getMotorMinDuty();

  /*
   * @brief set the decay of the PID output, to help it converge to 0 on periods
   * of inactivity (stationary robot)
   * @param geometric_decay is the decay factor. PID output *= geometric_decay
   * which happens every timestep
   */
  void setOutputDecay(float geometric_decay);

  /*
   * @brief get the decay of the PID output, to help it converge to 0 on periods
   * of inactivity (stationary robot)
   */
  float getOutputDecay();

  /*
   * @brief sets the open-loop reference for the motors, which is defined by max
   * rpm of the motor with 0 load
   * @param open_loop_max_motor_rpm is the
   */
  void setOpenLoopMaxRpm(float open_loop_max_motor_rpm);

  /*
   * @brief get the open-loop reference for the motors
   */
  float getOpenLoopMaxRpm();

  /*
   * @brief sets the curvature correction for both sides of the robot.
   * @param left_trim is a value on the range [0, 1] which scales (reduces) the
   * left side command
   * @param right_trim is a value on the range [0, 1] which scales (reduces) the
   * right side command
   */
  void setTrim(float left_trim, float right_trim);

  /*
   * @brief gets the curvature correction for the left side of the robot
   */
  float getLeftTrim();

  /*
   * @brief gets the curvature correction for the right side of the robot
   */
  float getRightTrim();

  /*
   * @brief Allows for a reduction in angular command based on current linear
   * velocity
   * @param angular_scaling_params is 2nd order polynomial coefficients for
   * scaling. The output is: final_angular_command = [ax**2 + bx + c] *
   * input_angular_command , where x is the current linear velocity
   */
  void setAngularScaling(angular_scaling_params angular_scaling_params);

  /*
   * @brief get the params of angular scaling
   */
  angular_scaling_params getAngularScaling();

  /*
   * @brief compute the duty cycles for each motor based on the target, current
   * speed, and current duty cycle
   */
  motor_data runMotionControl(robot_velocities velocity_targets,
                              motor_data current_duty_cycles,
                              motor_data current_motor_speeds);

  /*
   * @brief get an estimate of the robot's velocities from the motor_speeds
   * (derived)
   */
  robot_velocities getMeasuredVelocities(motor_data current_motor_speeds);

 private:
  std::string log_folder_path_;
#ifdef DEBUG
  std::ofstream log_file_;
#endif

  robot_motion_mode_t operating_mode_;
  robot_geometry robot_geometry_;

  std::mutex pid_mutex_;
  std::unique_ptr<PidController> pid_controller_left_;
  std::unique_ptr<PidController> pid_controller_right_;

  std::unique_ptr<PidController> pid_controller_fl_;
  std::unique_ptr<PidController> pid_controller_fr_;
  std::unique_ptr<PidController> pid_controller_rl_;
  std::unique_ptr<PidController> pid_controller_rr_;

  pid_gains pid_gains_;
  robot_velocities measured_velocities_;

  float open_loop_max_wheel_rpm_;

  float max_motor_duty_;
  float min_motor_duty_;

  float left_trim_value_;
  float right_trim_value_;

  float max_linear_acceleration_;
  float max_angular_acceleration_;

  angular_scaling_params angular_scaling_params_;

  float geometric_decay_;

  std::chrono::steady_clock::time_point time_last_;
  std::chrono::steady_clock::time_point time_origin_;

  motor_data duty_cycles_;

  void initializePids();

  motor_data computeMotorCommandsDual_(motor_data target_wheel_speeds,
                                       motor_data current_motor_speeds);

  motor_data computeMotorCommandsQuad_(motor_data target_wheel_speeds,
                                       motor_data current_motor_speeds);

  motor_data clipDutyCycles_(motor_data proposed_duties);

  motor_data computeTorqueDistribution_(motor_data current_motor_speeds,
                                        motor_data power_proposals);
};

// #TODO: implement if needed
class Control::AlphaBetaFilter {
 public:
  /* constructors */
  AlphaBetaFilter(float alpha);

  float update(float new_value);

 private:
  float running_sum_;
};
