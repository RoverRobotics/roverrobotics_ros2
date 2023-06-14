#include "utils.hpp"

namespace RoverRobotics {
PidGains::PidGains() {}

OdomControl::OdomControl()
    : MOTOR_MAX_VEL_(0.5),
      MOTOR_MIN_VEL_(0),
      MOTOR_DEADBAND_(9),
      MAX_ACCEL_CUTOFF_(5.0),
      fs_(nullptr),
      K_P_(0),
      K_I_(0),
      K_D_(0),
      velocity_filtered_history_(5, 0),
      velocity_history_(3, 0),
      use_control_(false),
      skip_measurement_(false),
      at_max_motor_speed_(false),
      at_min_motor_speed_(false),
      stop_integrating_(false),
      velocity_error_(0),
      integral_error_(0),
      differential_error_(0),
      velocity_commanded_(0),
      velocity_measured_(0),
      measured_vel_(0) {}

OdomControl::OdomControl(bool use_control, PidGains pid_gains, double max,
                         double min, std::ofstream* fs)
    : MOTOR_MAX_VEL_(max),
      MOTOR_MIN_VEL_(min),
      MOTOR_DEADBAND_(9),
      MAX_ACCEL_CUTOFF_(5.0),
      fs_(fs),
      K_P_(pid_gains.Kp),
      K_I_(pid_gains.Ki),
      K_D_(pid_gains.Kd),
      velocity_filtered_history_(5, 0),
      velocity_history_(3, 0),
      use_control_(use_control),
      skip_measurement_(false),
      at_max_motor_speed_(false),
      at_min_motor_speed_(false),
      stop_integrating_(false),
      velocity_error_(0),
      integral_error_(0),
      differential_error_(0),
      velocity_commanded_(0),
      velocity_measured_(0),
      measured_vel_(0) {
  if (fs_ != nullptr && fs_->is_open()) {
    *fs_ << "time,Kp,Ki,Kd,error,integral_error,differential_error,error_"
            "filtered,meas_vel,filt_vel,cmd_vel,dt,motor_cmd\n";
    fs_->flush();
  }
}

OdomControl::OdomControl(bool use_control, PidGains pid_gains, double max,
                         double min)
    : MOTOR_MAX_VEL_(max),
      MOTOR_MIN_VEL_(min),
      MOTOR_DEADBAND_(9),
      MAX_ACCEL_CUTOFF_(5.0),
      fs_(nullptr),
      K_P_(pid_gains.Kp),
      K_I_(pid_gains.Ki),
      K_D_(pid_gains.Kd),
      velocity_filtered_history_(5, 0),
      velocity_history_(3, 0),
      use_control_(use_control),
      skip_measurement_(false),
      at_max_motor_speed_(false),
      at_min_motor_speed_(false),
      stop_integrating_(false),
      velocity_error_(0),
      integral_error_(0),
      differential_error_(0),
      velocity_commanded_(0),
      velocity_measured_(0),
      measured_vel_(0) {}

double OdomControl::run(double commanded_vel, double measured_vel, double dt,
                        int firmwareBuildNumber) {
  velocity_commanded_ = commanded_vel;

  velocity_measured_ = measured_vel;

  /*
  Truncate the last two digits of the firmware version number,
  which returns in the format aabbcc. We divide by 100 to truncate
  cc since we don't care about the PATCH field of semantic versioning
  */
  int firmwareBuildNumberTrunc = firmwareBuildNumber / 100;

  // If stopping, stop now when velocity has slowed.
  if ((commanded_vel == 0.0) && (fabs(measured_vel) < 0.3)) {
    integral_error_ = 0;
    if (hasZeroHistory(velocity_filtered_history_)) {
      return 0;  // return 0 vel
    }
  }

  // If controller should be ON, run it.
  if (use_control_) {
    velocity_error_ = commanded_vel - measured_vel;

    if (!skip_measurement_) {
      double tempPID = PID(velocity_error_, dt);
      motor_command_vel_ = feedThroughControl() + tempPID;
    }
  } else {
    motor_command_vel_ = feedThroughControl();
  }
  return motor_command_vel_;
}

double OdomControl::feedThroughControl() { return velocity_commanded_; }

void OdomControl::reset() {
  integral_error_ = 0;
  velocity_error_ = 0;
  velocity_commanded_ = 0;
  velocity_measured_ = 0;
  measured_vel_ = 0;
  std::fill(velocity_filtered_history_.begin(),
            velocity_filtered_history_.end(), 0);
  motor_command_vel_ = 0;  // reset velocity
  skip_measurement_ = false;
}

double OdomControl::PID(double error, double dt) {
  double p_val = P(error);
  double i_val = I(error, dt);
  double d_val = D(error, dt);
  double pid_val = p_val + i_val + d_val;
  if (fabs(pid_val) > (MOTOR_MAX_VEL_))
  // Only integrate if the motor's aren't already at full speed
  {
    stop_integrating_ = true;
  } else {
    stop_integrating_ = false;
  }
  return pid_val;
}

double OdomControl::D(double error, double dt) {
  differential_error_ =
      (velocity_filtered_history_[0] - velocity_filtered_history_[1]) / dt;
  return K_D_ * differential_error_;
}

double OdomControl::I(double error, double dt) {
  if (!stop_integrating_) {
    integral_error_ += error * dt;
  }
  return K_I_ * integral_error_;
}

double OdomControl::P(double error) {
  double p_val = error * K_P_;
  return p_val;
}

bool OdomControl::hasZeroHistory(const std::vector<double>& vel_history) {
  double avg =
      (fabs(vel_history[0]) + fabs(vel_history[1]) + fabs(vel_history[2])) /
      3.0;
  if (avg < 0.03)
    return true;
  else
    return false;
}

double OdomControl::boundMotorSpeed(double motor_speed, double max,
                                    double min) {
  double test_motor;
  at_max_motor_speed_ = false;
  at_min_motor_speed_ = false;

  if (motor_speed > max) {
    motor_speed = max;
    at_max_motor_speed_ = true;
  }
  if (motor_speed < min) {
    motor_speed = min;
    at_min_motor_speed_ = true;
  }

  test_motor = motor_speed;

  return test_motor;
}

double OdomControl::deadbandOffset(double motor_speed, double deadband_offset) {
  // Compensate for deadband
  if (motor_speed >= 0) {
    return (motor_speed + deadband_offset);
  } else {
    return (motor_speed - deadband_offset);
  }
}

}  // namespace RoverRobotics
