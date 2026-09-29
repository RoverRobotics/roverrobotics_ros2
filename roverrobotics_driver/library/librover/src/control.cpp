#include "control.hpp"

#include <math.h>

#include <limits>

namespace Control {
/* functions */

motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry) {
  /* travel rate(m/s) */
  float left_travel_rate =
      target_velocities.linear_velocity -
      (0.5 * target_velocities.angular_velocity * robot_geometry.wheel_base);
  float right_travel_rate =
      target_velocities.linear_velocity +
      (0.5 * target_velocities.angular_velocity * robot_geometry.wheel_base);

  /* convert (m/s) -> rpm */
  float left_wheel_speed =
      (left_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;
  float right_wheel_speed =
      (right_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;
  motor_data returnstruct = {left_wheel_speed, right_wheel_speed,
                             left_wheel_speed, right_wheel_speed};
  return returnstruct;
}

robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry) {
  float left_magnitude = (wheel_speeds.fl + wheel_speeds.rl) / 2;
  float right_magnitude = (wheel_speeds.fr + wheel_speeds.rr) / 2;

  float left_travel_rate =
      left_magnitude * RPM_TO_RADS_SEC * robot_geometry.wheel_radius;
  float right_travel_rate =
      right_magnitude * RPM_TO_RADS_SEC * robot_geometry.wheel_radius;

  /* difference between left and right travel rates */
  float travel_differential = right_travel_rate - left_travel_rate;

  /* compute velocities */
  float linear_velocity = (right_travel_rate + left_travel_rate) / 2;
  float angular_velocity =
      travel_differential /
      robot_geometry.wheel_base;  // possibly add traction factor here

  robot_velocities returnstruct;
  returnstruct.linear_velocity = linear_velocity;
  returnstruct.angular_velocity = angular_velocity;
  return returnstruct;
}

robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt) {
  /* compute proposed acceleration */
  float linear_acceleration = (target_velocities.linear_velocity -
                               measured_velocities.linear_velocity) /
                              dt;
  float angular_acceleration = (target_velocities.angular_velocity -
                                measured_velocities.angular_velocity) /
                               dt;

  /* clip the proposed acceleration into an acceptable acceleration */
  if (std::abs(linear_acceleration) >
      std::abs(delta_v_limits.linear_velocity)) {
    std::signbit(linear_acceleration)
        ? linear_acceleration = -delta_v_limits.linear_velocity
        : linear_acceleration = delta_v_limits.linear_velocity;
  }

  /* TODO: fix this */
  if (std::abs(angular_acceleration) >
      std::abs(delta_v_limits.angular_velocity))
  {
    std::signbit(angular_acceleration)
        ? angular_acceleration = -delta_v_limits.angular_velocity
        : angular_acceleration = delta_v_limits.angular_velocity;
  }

  /* calculate new velocities */
  robot_velocities return_velocities;
  return_velocities.linear_velocity =
      measured_velocities.linear_velocity + linear_acceleration * dt;
  return_velocities.angular_velocity =
      measured_velocities.angular_velocity + angular_acceleration * dt;
  //return_velocities.angular_velocity = target_velocities.angular_velocity;
#ifdef DEBUG
  std::cerr << "target " << target_velocities.linear_velocity << std::endl;
  std::cerr << "measured " << measured_velocities.linear_velocity << std::endl;
  std::cerr << "return " << return_velocities.linear_velocity << std::endl;
  std::cerr << "linear acc " << linear_acceleration << std::endl;
#endif
  return return_velocities;
}

robot_velocities scaleAngularCommand(robot_velocities target_velocities,
                                     robot_velocities measured_velocities,
                                     angular_scaling_params scaling_params) {
  float angular_scale_factor = std::clamp(
      (float)(scaling_params.a_coef *
                  pow(measured_velocities.linear_velocity, 2) +
              scaling_params.b_coef * measured_velocities.linear_velocity +
              scaling_params.c_coef),
      scaling_params.min_scale_val, scaling_params.max_scale_val);

  return (robot_velocities){
      .linear_velocity = target_velocities.linear_velocity,
      .angular_velocity =
          target_velocities.angular_velocity * angular_scale_factor};
}

/* classes */
PidController::PidController(struct pid_gains pid_gains, std::string name)
    : /* defaults */
      integral_error_(0),
      previous_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      pos_max_output_(std::numeric_limits<float>::max()),
      neg_max_output_(std::numeric_limits<float>::lowest()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  name_ = name;
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

PidController::PidController(struct pid_gains pid_gains,
                             pid_output_limits pid_output_limits,
                             std::string name)
    : /* defaults */
      integral_error_(0),
      previous_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  name_ = name;
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
};

void PidController::setGains(struct pid_gains pid_gains) {
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

pid_gains PidController::getGains() {
  pid_gains pid_gains;
  pid_gains.kp = kp_;
  pid_gains.ki = ki_;
  pid_gains.kd = kd_;
  return pid_gains;
}
void SkidRobotMotionController::setWheelTrims(float fl, float fr, float rl, float rr) {
  trim_fl_ = fl;
  trim_fr_ = fr;
  trim_rl_ = rl;
  trim_rr_ = rr;
}
void SkidRobotMotionController::setTrim(float left_trim, float right_trim) {
  left_trim_value_ = left_trim;
  right_trim_value_ = right_trim;
  /* runMotionControl() reads the per-wheel trims */
  trim_fl_ = left_trim;
  trim_rl_ = left_trim;
  trim_fr_ = right_trim;
  trim_rr_ = right_trim;
}
float SkidRobotMotionController::getLeftTrim() {
  return left_trim_value_;
}
float SkidRobotMotionController::getRightTrim() {
  return right_trim_value_;
}

void PidController::setOutputLimits(pid_output_limits pid_output_limits) {
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
}

pid_output_limits PidController::getOutputLimits() {
  pid_output_limits returnstruct;
  returnstruct.posmax = pos_max_output_;
  returnstruct.negmax = neg_max_output_;
  return returnstruct;
}

void PidController::setIntegralErrorLimit(float error_limit) {
  integral_error_limit_ = error_limit;
}

float PidController::getIntegralErrorLimit() { return integral_error_limit_; }

void PidController::reset() {
  integral_error_ = 0;
  previous_error_ = 0;
  time_last_ = std::chrono::steady_clock::now();
}

void PidController::writePidDataToCsv(std::ofstream &log_file,
                                      pid_outputs data) {
  log_file << "pid," << data.name << "," << data.time << ","
           << data.target_value << "," << data.measured_value << ","
           << data.pid_output << "," << data.error << "," << data.integral_error
           << "," << data.delta_error << "," << data.kp << "," << data.ki << ","
           << data.kd << "," << std::endl;
  log_file.flush();
}

pid_outputs PidController::runControl(float target, float measured) {
  /* current time */
  std::chrono::steady_clock::time_point time_now =
      std::chrono::steady_clock::now();

  /* delta time (S) */
  float delta_time =
      std::chrono::duration<float>(time_now - time_last_).count();

  /* update time bookkeeping */
  time_last_ = time_now;

  /* error */
  float error = target - measured;

  /* integrate */
  integral_error_ += error * delta_time;

  /* differentiate */
  float delta_error = error - previous_error_;

  /* clip integral error */
  integral_error_ = std::clamp(integral_error_, -integral_error_limit_,
                               integral_error_limit_);

  /* P I D terms */
  float p = kp_ * error;
  float i = ki_ * integral_error_;
  float d = kd_ * (delta_error / delta_time);

  /* compute output */
  float output = p + i + d;

  /* clip output */
  output = std::clamp(output, neg_max_output_, pos_max_output_);

  pid_outputs returnstruct;
  returnstruct.pid_output = output;
  returnstruct.name = name_;
  returnstruct.dt = delta_time;
  returnstruct.time =
      std::chrono::duration<double>(time_now - time_origin_).count();
  returnstruct.error = error;
  returnstruct.integral_error = integral_error_;
  returnstruct.delta_error = (delta_error / delta_time);
  returnstruct.target_value = target;
  returnstruct.measured_value = measured;
  returnstruct.kp = kp_;
  returnstruct.ki = ki_;
  returnstruct.kd = kd_;

  previous_error_ = error;
  return returnstruct;
}

SkidRobotMotionController::SkidRobotMotionController() {}
SkidRobotMotionController::SkidRobotMotionController(
    robot_motion_mode_t operating_mode, robot_geometry robot_geometry,
    float max_motor_duty, float min_motor_duty, float left_trim,
    float right_trim, float open_loop_max_wheel_rpm)
    : log_folder_path_("~/Documents/"),
      duty_cycles_({0}),
      measured_velocities_({0}),
      angular_scaling_params_((angular_scaling_params){.a_coef = 0,
                                                       .b_coef = 0,
                                                       .c_coef = 1,
                                                       .min_scale_val = 1.0,
                                                       .max_scale_val = 1.0}),
      max_linear_acceleration_(std::numeric_limits<float>::max()),
      max_angular_acceleration_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  open_loop_max_wheel_rpm_ = open_loop_max_wheel_rpm;
  min_motor_duty_ = min_motor_duty;
  max_motor_duty_ = max_motor_duty;
  left_trim_value_ = left_trim;
  right_trim_value_ = right_trim;
  operating_mode_ = operating_mode;
  robot_geometry_ = robot_geometry;
#ifdef DEBUG
  /*open a log file to store control data*/
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  auto filename = oss.str();

  log_file_.open("/home/rover/Documents/" + filename + ".csv");
  log_file_ << "type,"
            << "name,"
            << "time,"
            << "col0,"
            << "col1,"
            << "col2,"
            << "col3,"
            << "col4,"
            << "col5,"
            << "col6,"
            << "col7,"
            << "col8,"
            << "col9,"
            << "col10,"
            << "col11," << std::endl;
  log_file_.flush();
#endif
}

SkidRobotMotionController::SkidRobotMotionController(
    robot_motion_mode_t operating_mode, robot_geometry robot_geometry,
    pid_gains pid_gains, float max_motor_duty, float min_motor_duty,
    float left_trim, float right_trim, float geometric_decay,
    float rest_wheel_rpm, float brake_band_duty, float brake_band_rpm,
    float rpm_per_duty)
    : log_folder_path_("~/Documents/"),
      duty_cycles_({0}),
      measured_velocities_({0}),
      angular_scaling_params_((angular_scaling_params){.a_coef = 0,
                                                       .b_coef = 0,
                                                       .c_coef = 1,
                                                       .min_scale_val = 1.0,
                                                       .max_scale_val = 1.0}),
      max_linear_acceleration_(std::numeric_limits<float>::max()),
      max_angular_acceleration_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
#ifdef DEBUG
  /*open a log file to store control data*/
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  auto filename = oss.str();
  std::cerr << "log file name " << filename + ".csv" << std::endl;
  log_file_.open("/home/rover/Documents/" + filename + ".csv");
  log_file_ << "type,"
            << "name,"
            << "time,"
            << "col0,"
            << "col1,"
            << "col2,"
            << "col3,"
            << "col4,"
            << "col5,"
            << "col6,"
            << "col7,"
            << "col8,"
            << "col9,"
            << "col10,"
            << "col11," << std::endl;
  log_file_.flush();
#endif

  operating_mode_ = operating_mode;
  robot_geometry_ = robot_geometry;
  pid_gains_ = pid_gains;
  max_motor_duty_ = max_motor_duty;
  min_motor_duty_ = min_motor_duty;
  left_trim_value_ = left_trim;
  right_trim_value_ = right_trim;
  geometric_decay_ = geometric_decay;
  rest_wheel_rpm_ = rest_wheel_rpm;
  brake_band_duty_ = brake_band_duty;
  brake_band_rpm_ = brake_band_rpm;
  if (rpm_per_duty > 0.0f) rpm_per_duty_ = rpm_per_duty;
  if (brake_band_duty_ > 0.0f && operating_mode_ == TRACTION_CONTROL) {
    std::cerr << "brake_band_duty ignored in TRACTION_CONTROL" << std::endl;
  }

  initializePids();
}

void SkidRobotMotionController::resetStopState() {
  std::lock_guard<std::mutex> lock(pid_mutex_);
  duty_cycles_ = {0, 0, 0, 0};
  /* only the pids of the active operating mode exist */
  for (auto *pid : {&pid_controller_fl_, &pid_controller_fr_, &pid_controller_rl_,
                    &pid_controller_rr_, &pid_controller_left_,
                    &pid_controller_right_}) {
    if (*pid) (*pid)->reset();
  }
  for (int i = 0; i < 4; i++) {
    brake_off_[i] = false;
    brake_scale_[i] = 1.0f;
    brake_collapse_[i] = false;
    brake_min_[i] = std::numeric_limits<float>::max();
    brake_stall_[i] = 0;
  }
  ff_prev_ = {0, 0, 0, 0};
  ff_vel_ = {0, 0};
  for (int i = 0; i < 4; i++) launch_armed_[i] = false;
  speed_filter_primed_ = false;
}

void SkidRobotMotionController::limitBrakeDuty_(bool stop, motor_data target,
                                                motor_data rpm) {
  float *duty[4] = {&duty_cycles_.fl, &duty_cycles_.fr, &duty_cycles_.rl,
                    &duty_cycles_.rr};
  const float w[4] = {rpm.fl, rpm.fr, rpm.rl, rpm.rr};
  const float tg[4] = {target.fl, target.fr, target.rl, target.rr};
  for (int i = 0; i < 4; i++) {
    const float speed = std::abs(w[i]);
    const float dir = std::copysign(1.0f, w[i]);
    /* not rolling faster than commanded: no braking episode */
    if (speed < rest_wheel_rpm_ || dir * tg[i] >= speed - BRAKE_ENTRY_RPM_) {
      brake_min_[i] = speed;
      brake_stall_[i] = 0;
      brake_scale_[i] = 1.0f;
      brake_off_[i] = false;
      brake_collapse_[i] = false;
      continue;
    }
    if (speed < brake_min_[i] - 1.0f) {
      brake_min_[i] = speed;
      brake_stall_[i] = 0;
    } else {
      brake_stall_[i]++;
    }
    /* speeding up (downhill): hand the wheel back to the PID */
    if (speed > brake_min_[i] + STOP_REGROW_RPM_) brake_off_[i] = true;
    if (brake_off_[i]) continue;
    /* not slowing: at low speed hand back to the PID, at speed widen the band step by step */
    if (brake_stall_[i] >= STOP_STALL_TICKS_) {
      brake_stall_[i] = 0;
      if (speed < BRAKE_ESCAPE_RPM_) brake_off_[i] = true;
      if (brake_scale_[i] >= BRAKE_WIDEN_MAX_) brake_collapse_[i] = true;
      brake_scale_[i] = std::min(brake_scale_[i] * BRAKE_WIDEN_, BRAKE_WIDEN_MAX_);
    }
    if (brake_off_[i]) continue;
    float band = brake_band_duty_ * brake_scale_[i];
    if (brake_band_rpm_ > 0.0f && speed > brake_band_rpm_)
      band *= brake_band_rpm_ / speed;
    float floor = brake_collapse_[i] ? 0.0f : speed / rpm_per_duty_ - band;
    /* on a stop never plug a rolling wheel */
    if (stop) floor = std::max(floor, 0.0f);
    floor = std::min(floor, max_motor_duty_);
    if (dir * *duty[i] < floor) *duty[i] = dir * floor;
  }
}

void SkidRobotMotionController::initializePids() {
  pid_mutex_.lock();
  switch (operating_mode_) {
    case OPEN_LOOP:
      break;
    case INDEPENDENT_WHEEL:
      /* one pid per wheel */
      pid_controller_fl_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_left");
      pid_controller_fr_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_right");
      pid_controller_rl_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_left");
      pid_controller_rr_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_right");
      break;
    case TRACTION_CONTROL:
      /* one pid per side */
      pid_controller_left_ =
          std::make_unique<PidController>(pid_gains_, "pid_left");
      pid_controller_right_ =
          std::make_unique<PidController>(pid_gains_, "pid_right");
      break;
    default:
      /* probably throw exception here */
      break;
  }
  pid_mutex_.unlock();
}
void SkidRobotMotionController::setAccelerationLimits(robot_velocities limits) {
  max_linear_acceleration_ = limits.linear_velocity;
  max_angular_acceleration_ = limits.angular_velocity;
}

robot_velocities SkidRobotMotionController::getAccelerationLimits() {
  robot_velocities returnstruct;
  returnstruct.angular_velocity = max_angular_acceleration_;
  returnstruct.linear_velocity = max_linear_acceleration_;
  return returnstruct;
}

void SkidRobotMotionController::setOperatingMode(
    robot_motion_mode_t operating_mode) {
  operating_mode_ = operating_mode;
  initializePids();
}

robot_motion_mode_t SkidRobotMotionController::getOperatingMode() {
  return operating_mode_;
}

void SkidRobotMotionController::setRobotGeometry(
    robot_geometry robot_geometry) {
  robot_geometry_ = robot_geometry;
}

robot_geometry SkidRobotMotionController::getRobotGeometry() {
  return robot_geometry_;
}

void SkidRobotMotionController::setPidGains(pid_gains pid_gains) {
  pid_gains_ = pid_gains;
}

pid_gains SkidRobotMotionController::getPidGains() { return pid_gains_; }

void SkidRobotMotionController::setMotorMaxDuty(float max_motor_duty) {
  max_motor_duty_ = max_motor_duty;
}
float SkidRobotMotionController::getMotorMaxDuty() { return max_motor_duty_; }

void SkidRobotMotionController::setMotorMinDuty(float min_motor_duty) {
  min_motor_duty_ = min_motor_duty;
}
float SkidRobotMotionController::getMotorMinDuty() { return min_motor_duty_; }

void SkidRobotMotionController::setFeedforward(float rpm_per_duty,
                                               float static_duty,
                                               float turn_duty) {
  std::lock_guard<std::mutex> lock(pid_mutex_);
  ff_rpm_per_duty_ = rpm_per_duty;
  ff_static_duty_ = static_duty;
  ff_turn_duty_ = turn_duty;
}

void SkidRobotMotionController::setFeedforwardVoltage(float calibration_voltage) {
  std::lock_guard<std::mutex> lock(pid_mutex_);
  ff_cal_voltage_ = calibration_voltage;
}

void SkidRobotMotionController::setBusVoltage(float volts) { bus_voltage_ = volts; }

void SkidRobotMotionController::setLowSpeedTrust(float rpm) {
  std::lock_guard<std::mutex> lock(pid_mutex_);
  low_speed_trust_rpm_ = rpm;
}

void SkidRobotMotionController::setSpeedFilter(float filter) {
  std::lock_guard<std::mutex> lock(pid_mutex_);
  speed_filter_ = filter;
  speed_filter_primed_ = false;
}

motor_data SkidRobotMotionController::feedforward_(motor_data targets,
                                                   float angular_velocity,
                                                   motor_data measured) {
  motor_data ff = {0, 0, 0, 0};
  if (ff_rpm_per_duty_ <= 0.0f) return ff;
  /* turn boost acts on each side's turning share only, so it helps a pivot but adds no forward push in an arc */
  const float left = 0.5f * (targets.fl + targets.rl);
  const float right = 0.5f * (targets.fr + targets.rr);
  const float linear = 0.5f * (left + right);
  const float turn_rpm = 0.5f * (right - left);
  const float pivot_share =
      std::abs(turn_rpm) / (std::abs(linear) + std::abs(turn_rpm) + 1e-3f);
  const float boost = ff_turn_duty_ * pivot_share *
      std::min(std::abs(angular_velocity) / FF_TURN_FULL_RADPS_, 1.0f);
  float *out[4] = {&ff.fl, &ff.fr, &ff.rl, &ff.rr};
  const float tg[4] = {targets.fl, targets.fr, targets.rl, targets.rr};
  const float side[4] = {-turn_rpm, turn_rpm, -turn_rpm, turn_rpm};
  const float w[4] = {measured.fl, measured.fr, measured.rl, measured.rr};
  /* the same duty gives proportionally more speed at a higher bus voltage */
  float vscale = 1.0f;
  const float vbus = bus_voltage_;
  if (ff_cal_voltage_ > 0.0f && vbus > 20.0f && vbus < 60.0f)
    vscale = std::clamp(ff_cal_voltage_ / vbus, FF_VSCALE_MIN_, FF_VSCALE_MAX_);
  for (int i = 0; i < 4; i++) {
    if (std::abs(tg[i]) < FF_MIN_TARGET_RPM_) continue;
    /* the boost is a breakaway assist: full while the wheel is stuck, gone once it reaches its target */
    const float fade = std::clamp(1.0f - std::abs(w[i]) / std::abs(tg[i]), 0.0f, 1.0f);
    *out[i] = vscale * (std::copysign(ff_static_duty_ + std::abs(tg[i]) / ff_rpm_per_duty_, tg[i]) +
                        std::copysign(boost * fade, side[i]));
  }
  return ff;
}

void SkidRobotMotionController::setOutputDecay(float geometric_decay) {
  geometric_decay_ = geometric_decay;
}
float SkidRobotMotionController::getOutputDecay() { return geometric_decay_; }

void SkidRobotMotionController::setOpenLoopMaxRpm(
    float open_loop_max_wheel_rpm) {
  open_loop_max_wheel_rpm_ = open_loop_max_wheel_rpm;
}
float SkidRobotMotionController::getOpenLoopMaxRpm() {
  return open_loop_max_wheel_rpm_;
}

void SkidRobotMotionController::setAngularScaling(
    angular_scaling_params angular_scaling_params) {
  angular_scaling_params_ = angular_scaling_params;
}

angular_scaling_params SkidRobotMotionController::getAngularScaling() {
  return angular_scaling_params_;
}

motor_data SkidRobotMotionController::computeMotorCommandsDual_(
    motor_data target_wheel_speeds, motor_data current_wheel_speeds) {
  /* average front and rear wheels */
  float left_magnitude =
      (current_wheel_speeds.fl + current_wheel_speeds.rl) / 2;
  float right_magnitude =
      (current_wheel_speeds.fr + current_wheel_speeds.rr) / 2;

  /* run pid, 1 per side */

  pid_mutex_.lock();
  pid_outputs l_pid_output =
      pid_controller_left_->runControl(target_wheel_speeds.fl, left_magnitude);

  pid_outputs r_pid_output = pid_controller_right_->runControl(
      target_wheel_speeds.fr, right_magnitude);
  pid_mutex_.unlock();

#ifdef DEBUG
  pid_controller_left_->writePidDataToCsv(log_file_, l_pid_output);
  pid_controller_right_->writePidDataToCsv(log_file_, r_pid_output);
#endif

  /* math to split the torque distribution */
  motor_data power_proposals = (motor_data){.fl = l_pid_output.pid_output,
                                            .fr = r_pid_output.pid_output,
                                            .rl = l_pid_output.pid_output,
                                            .rr = r_pid_output.pid_output};

  isnan(power_proposals.fr) ? power_proposals.fr = 0
                            : power_proposals.fr = power_proposals.fr;
  isnan(power_proposals.fl) ? power_proposals.fl = 0
                            : power_proposals.fl = power_proposals.fl;
  isnan(power_proposals.rr) ? power_proposals.rr = 0
                            : power_proposals.rr = power_proposals.rr;
  isnan(power_proposals.rl) ? power_proposals.rl = 0
                            : power_proposals.rl = power_proposals.rl;

  /* add here */

  return power_proposals;
}

motor_data SkidRobotMotionController::computeMotorCommandsQuad_(
    motor_data target_wheel_speeds, motor_data current_wheel_speeds) {
  /* run pid, 1 per wheel */
  pid_mutex_.lock();
  pid_outputs fl_pid_output = pid_controller_fl_->runControl(
      target_wheel_speeds.fl, current_wheel_speeds.fl);

  pid_outputs fr_pid_output = pid_controller_fr_->runControl(
      target_wheel_speeds.fr, current_wheel_speeds.fr);

  pid_outputs rl_pid_output = pid_controller_rl_->runControl(
      target_wheel_speeds.rl, current_wheel_speeds.rl);

  pid_outputs rr_pid_output = pid_controller_rr_->runControl(
      target_wheel_speeds.rr, current_wheel_speeds.rr);
  pid_mutex_.unlock();
#ifdef DEBUG
  pid_controller_fl_->writePidDataToCsv(log_file_, fl_pid_output);
  pid_controller_fr_->writePidDataToCsv(log_file_, fr_pid_output);
  pid_controller_rl_->writePidDataToCsv(log_file_, rl_pid_output);
  pid_controller_rr_->writePidDataToCsv(log_file_, rr_pid_output);
#endif

  /* math to split the torque distribution */
  motor_data power_proposals = (motor_data){.fl = fl_pid_output.pid_output,
                                            .fr = fr_pid_output.pid_output,
                                            .rl = rl_pid_output.pid_output,
                                            .rr = rr_pid_output.pid_output};

  isnan(power_proposals.fr) ? power_proposals.fr = 0
                            : power_proposals.fr = power_proposals.fr;
  isnan(power_proposals.fl) ? power_proposals.fl = 0
                            : power_proposals.fl = power_proposals.fl;
  isnan(power_proposals.rr) ? power_proposals.rr = 0
                            : power_proposals.rr = power_proposals.rr;
  isnan(power_proposals.rl) ? power_proposals.rl = 0
                            : power_proposals.rl = power_proposals.rl;

  /* add here */

  return power_proposals;
}

motor_data SkidRobotMotionController::clipDutyCycles_(
    motor_data proposed_duties) {
  /* clip extreme duty cycles in either direction (positive or negative) */
  proposed_duties.fr =
      std::clamp(proposed_duties.fr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.fl =
      std::clamp(proposed_duties.fl, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rr =
      std::clamp(proposed_duties.rr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rl =
      std::clamp(proposed_duties.rl, -max_motor_duty_, max_motor_duty_);

  /* enforce minimum magnitude (positive or negative) */
  if (std::abs(proposed_duties.fl) < min_motor_duty_) proposed_duties.fl = 0;
  if (std::abs(proposed_duties.fr) < min_motor_duty_) proposed_duties.fr = 0;
  if (std::abs(proposed_duties.rl) < min_motor_duty_) proposed_duties.rl = 0;
  if (std::abs(proposed_duties.rr) < min_motor_duty_) proposed_duties.rr = 0;

  return proposed_duties;
}

motor_data SkidRobotMotionController::computeTorqueDistribution_(
    motor_data current_wheel_speeds, motor_data power_proposals) {
  /* right side */
  /* if both wheels are moving then ... */

  /* if front wheel is spinning faster ... */
  if (std::abs(current_wheel_speeds.fr) >= std::abs(current_wheel_speeds.rr)) {
    /* scale down FRONT RIGHT power */
    power_proposals.fr *=
        (isnan(std::abs(current_wheel_speeds.rr / current_wheel_speeds.fr))
             ? 1.0
             : std::abs(current_wheel_speeds.rr / current_wheel_speeds.fr));
  } else {
    /* scale down REAR RIGHT power */
    power_proposals.rr *=
        (isnan(std::abs(current_wheel_speeds.fr / current_wheel_speeds.rr))
             ? 1.0
             : std::abs(current_wheel_speeds.fr / current_wheel_speeds.rr));
  }

  /* left side */
  if (std::abs(current_wheel_speeds.fl) >= std::abs(current_wheel_speeds.rl)) {
    /* scale down FRONT LEFT power */
    power_proposals.fl *=
        (isnan(std::abs(current_wheel_speeds.rl / current_wheel_speeds.fl))
             ? 1.0
             : std::abs(current_wheel_speeds.rl / current_wheel_speeds.fl));
  } else {
    /* scale down REAR LEFT power */
    power_proposals.rl *=
        (isnan(std::abs(current_wheel_speeds.fl / current_wheel_speeds.rl))
             ? 1.0
             : std::abs(current_wheel_speeds.fl / current_wheel_speeds.rl));
  }

  return power_proposals;
}

robot_velocities SkidRobotMotionController::getMeasuredVelocities(
    motor_data current_wheel_speeds) {
  return computeVelocitiesFromWheelspeeds(current_wheel_speeds,
                                          robot_geometry_);
}

motor_data SkidRobotMotionController::runMotionControl(
    robot_velocities velocity_targets, motor_data current_duty_cycles,
    motor_data current_wheel_speeds) {
  /* take the time*/
  std::chrono::steady_clock::time_point time_now =
      std::chrono::steady_clock::now();

  /* delta time (S) */
  float delta_time =
      std::chrono::duration<float>(time_now - time_last_).count();

  float accumulated_time =
      std::chrono::duration<float>(time_now - time_origin_).count();

  time_last_ = time_now;

  /* get estimated robot velocities */
  measured_velocities_ =
      computeVelocitiesFromWheelspeeds(current_wheel_speeds, robot_geometry_);

  /* limit acceleration */
  robot_velocities velocity_commands;
  robot_velocities acceleration_limits = {max_linear_acceleration_,
                                          max_angular_acceleration_};

  velocity_commands = limitAcceleration(velocity_targets, measured_velocities_,
                                        acceleration_limits, delta_time);

  /* scale the angular command */
  velocity_commands = scaleAngularCommand(
      velocity_commands, measured_velocities_, angular_scaling_params_);

  /* get target wheelspeeds from velocities */
  motor_data target_wheel_speeds =
      computeSkidSteerWheelSpeeds(velocity_commands, robot_geometry_);

  /* apply trim value to targets */
  target_wheel_speeds.fl *= trim_fl_;
  target_wheel_speeds.fr *= trim_fr_;
  target_wheel_speeds.rl *= trim_rl_;
  target_wheel_speeds.rr *= trim_rr_;

  /* do control */
  motor_data motor_duties_add;
  motor_data modified_duties = {0, 0, 0, 0};
  switch (operating_mode_) {
    case OPEN_LOOP:
      duty_cycles_.fr = target_wheel_speeds.fr / open_loop_max_wheel_rpm_;
      duty_cycles_.fl = target_wheel_speeds.fl / open_loop_max_wheel_rpm_;
      duty_cycles_.rr = target_wheel_speeds.rr / open_loop_max_wheel_rpm_;
      duty_cycles_.rl = target_wheel_speeds.rl / open_loop_max_wheel_rpm_;

      /* don't allow duties higher than the limits */
      modified_duties = clipDutyCycles_(duty_cycles_);

      break;

    case INDEPENDENT_WHEEL: {
      motor_data feedback = current_wheel_speeds;
      if (speed_filter_ > 0.0f) {
        if (!speed_filter_primed_) {
          speed_filtered_ = current_wheel_speeds;
          speed_filter_primed_ = true;
        }
        const float f = speed_filter_;
        speed_filtered_.fl = f * speed_filtered_.fl + (1.0f - f) * current_wheel_speeds.fl;
        speed_filtered_.fr = f * speed_filtered_.fr + (1.0f - f) * current_wheel_speeds.fr;
        speed_filtered_.rl = f * speed_filtered_.rl + (1.0f - f) * current_wheel_speeds.rl;
        speed_filtered_.rr = f * speed_filtered_.rr + (1.0f - f) * current_wheel_speeds.rr;
        feedback = speed_filtered_;
      }
      /* the accumulator holds only the PID correction; feedforward is re-added fresh each cycle.
         feedforward follows the requested command, never the measured-anchored limiter output */
      const float ff_dt = std::min(delta_time, 0.1f);
      ff_vel_.linear_velocity += std::clamp(
          velocity_targets.linear_velocity - ff_vel_.linear_velocity,
          -max_linear_acceleration_ * ff_dt, max_linear_acceleration_ * ff_dt);
      ff_vel_.angular_velocity += std::clamp(
          velocity_targets.angular_velocity - ff_vel_.angular_velocity,
          -max_angular_acceleration_ * ff_dt, max_angular_acceleration_ * ff_dt);
      motor_data ff_targets =
          computeSkidSteerWheelSpeeds(ff_vel_, robot_geometry_);
      ff_targets.fl *= trim_fl_;
      ff_targets.fr *= trim_fr_;
      ff_targets.rl *= trim_rl_;
      ff_targets.rr *= trim_rr_;
      motor_data ff = feedforward_(ff_targets, ff_vel_.angular_velocity,
                                   current_wheel_speeds);
      motor_duties_add = computeMotorCommandsQuad_(target_wheel_speeds, feedback);
      /* while the feedforward is still ramping it carries the move alone; the PID only corrects at steady command */
      const bool ff_ramping =
          ff_rpm_per_duty_ > 0.0f &&
          (std::abs(velocity_targets.linear_velocity - ff_vel_.linear_velocity) > 1e-4f ||
           std::abs(velocity_targets.angular_velocity - ff_vel_.angular_velocity) > 1e-4f);
      if (ff_ramping) motor_duties_add = {0, 0, 0, 0};
      /* and it stays out per wheel until that wheel has actually caught up with the new target */
      if (ff_rpm_per_duty_ > 0.0f) {
        const float tg[4] = {ff_targets.fl, ff_targets.fr, ff_targets.rl, ff_targets.rr};
        const float w[4] = {current_wheel_speeds.fl, current_wheel_speeds.fr,
                            current_wheel_speeds.rl, current_wheel_speeds.rr};
        float *add[4] = {&motor_duties_add.fl, &motor_duties_add.fr, &motor_duties_add.rl,
                         &motor_duties_add.rr};
        for (int i = 0; i < 4; i++) {
          /* arm on any real target change: a small step can finish ramping within one cycle */
          const bool stepped = std::abs(tg[i] - launch_prev_tg_[i]) > LAUNCH_TARGET_STEP_RPM_;
          launch_prev_tg_[i] = tg[i];
          if ((ff_ramping || stepped) && !launch_armed_[i]) {
            launch_armed_[i] = true;
            launch_t_[i] = 0.0f;
          }
          const bool low = std::abs(tg[i]) >= FF_MIN_TARGET_RPM_ &&
                           std::abs(tg[i]) < low_speed_trust_rpm_;
          if (!launch_armed_[i]) {
            if (low) *add[i] *= LOW_SPEED_PID_SCALE_;
            continue;
          }
          const float progress =
              std::abs(tg[i]) < FF_MIN_TARGET_RPM_ ? 1.0f : w[i] / tg[i];
          if ((!low && progress >= LAUNCH_PROGRESS_) || launch_t_[i] > LAUNCH_MAX_S_) {
            launch_armed_[i] = false;
            if (low) *add[i] *= LOW_SPEED_PID_SCALE_;
            continue;
          }
          *add[i] = 0.0f;
          launch_t_[i] += ff_dt;
        }
      }
      duty_cycles_.fl = (duty_cycles_.fl - ff_prev_.fl + motor_duties_add.fl) * geometric_decay_ + ff.fl;
      duty_cycles_.fr = (duty_cycles_.fr - ff_prev_.fr + motor_duties_add.fr) * geometric_decay_ + ff.fr;
      duty_cycles_.rr = (duty_cycles_.rr - ff_prev_.rr + motor_duties_add.rr) * geometric_decay_ + ff.rr;
      duty_cycles_.rl = (duty_cycles_.rl - ff_prev_.rl + motor_duties_add.rl) * geometric_decay_ + ff.rl;
      ff_prev_ = ff;

      /* clamp the state, not just the output, so it cannot wind up */
      duty_cycles_.fl = std::clamp(duty_cycles_.fl, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.fr = std::clamp(duty_cycles_.fr, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.rr = std::clamp(duty_cycles_.rr, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.rl = std::clamp(duty_cycles_.rl, -max_motor_duty_, max_motor_duty_);

      if (brake_band_duty_ > 0.0f) {
        motor_data raw_targets =
            computeSkidSteerWheelSpeeds(velocity_targets, robot_geometry_);
        raw_targets.fl *= trim_fl_;
        raw_targets.fr *= trim_fr_;
        raw_targets.rl *= trim_rl_;
        raw_targets.rr *= trim_rr_;
        limitBrakeDuty_(isStopCommand_(velocity_targets), raw_targets,
                        current_wheel_speeds);
      }

      /* with feedforward, leftover correction must never drive a rolling wheel backwards on a stop */
      if (ff_rpm_per_duty_ > 0.0f && isStopCommand_(velocity_targets)) {
        float *d[4] = {&duty_cycles_.fl, &duty_cycles_.fr, &duty_cycles_.rl,
                       &duty_cycles_.rr};
        const float w[4] = {current_wheel_speeds.fl, current_wheel_speeds.fr,
                            current_wheel_speeds.rl, current_wheel_speeds.rr};
        for (int i = 0; i < 4; i++)
          if (std::abs(w[i]) >= rest_wheel_rpm_ && *d[i] * w[i] < 0.0f) *d[i] = 0.0f;
      }

      /* on a commanded stop, release each nearly still wheel so leftover duty can't rock it */
      if (isStopCommand_(velocity_targets)) {
        pid_mutex_.lock();
        if (std::abs(current_wheel_speeds.fl) < rest_wheel_rpm_) {
          duty_cycles_.fl = 0;
          pid_controller_fl_->reset();
        }
        if (std::abs(current_wheel_speeds.fr) < rest_wheel_rpm_) {
          duty_cycles_.fr = 0;
          pid_controller_fr_->reset();
        }
        if (std::abs(current_wheel_speeds.rl) < rest_wheel_rpm_) {
          duty_cycles_.rl = 0;
          pid_controller_rl_->reset();
        }
        if (std::abs(current_wheel_speeds.rr) < rest_wheel_rpm_) {
          duty_cycles_.rr = 0;
          pid_controller_rr_->reset();
        }
        pid_mutex_.unlock();
      }

      /* don't allow duties higher or lower than the limits */
      modified_duties = clipDutyCycles_(duty_cycles_);

      /* with feedforward, the min-duty cut (a VESC brake) applies only to wheels that should be stopped */
      if (ff_rpm_per_duty_ > 0.0f) {
        const float *ffw[4] = {&ff_prev_.fl, &ff_prev_.fr, &ff_prev_.rl, &ff_prev_.rr};
        const float *raw[4] = {&duty_cycles_.fl, &duty_cycles_.fr, &duty_cycles_.rl,
                               &duty_cycles_.rr};
        float *out[4] = {&modified_duties.fl, &modified_duties.fr, &modified_duties.rl,
                         &modified_duties.rr};
        for (int i = 0; i < 4; i++)
          if (*ffw[i] != 0.0f && *out[i] == 0.0f)
            *out[i] = std::clamp(*raw[i], -max_motor_duty_, max_motor_duty_);
      }

      break;
    }

    case TRACTION_CONTROL:
      /* determine how much change is needed to the duty cycles */
      motor_duties_add =
          computeMotorCommandsDual_(target_wheel_speeds, current_wheel_speeds);

      /* add the change to the duty cycles */
      duty_cycles_.fl += motor_duties_add.fl;
      duty_cycles_.fr += motor_duties_add.fr;
      duty_cycles_.rr += motor_duties_add.rr;
      duty_cycles_.rl += motor_duties_add.rl;

      /* add a geometric decay to the duty cycles */
      duty_cycles_.fl *= geometric_decay_;
      duty_cycles_.fr *= geometric_decay_;
      duty_cycles_.rr *= geometric_decay_;
      duty_cycles_.rl *= geometric_decay_;

      duty_cycles_.fl = std::clamp(duty_cycles_.fl, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.fr = std::clamp(duty_cycles_.fr, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.rr = std::clamp(duty_cycles_.rr, -max_motor_duty_, max_motor_duty_);
      duty_cycles_.rl = std::clamp(duty_cycles_.rl, -max_motor_duty_, max_motor_duty_);

      /* run traction control */
      modified_duties =
          computeTorqueDistribution_(current_wheel_speeds, duty_cycles_);

      /* don't allow duties higher or lower than the limits */
      modified_duties = clipDutyCycles_(modified_duties);

      break;

    default:
      std::cerr << "invalid motion control type.. commanding 0 motion"
                << std::endl;
      duty_cycles_ = {0, 0, 0, 0};
      break;
  }

#ifdef DEBUG
  log_file_ << "motion,"
            << "skid," << accumulated_time << ","
            << velocity_commands.linear_velocity << ","
            << velocity_commands.angular_velocity << ","
            << measured_velocities_.linear_velocity << ","
            << measured_velocities_.angular_velocity << ","
            << current_wheel_speeds.fl << "," << current_wheel_speeds.fr << ","
            << current_wheel_speeds.rl << "," << current_wheel_speeds.rr << ","
            << duty_cycles_.fl << "," << duty_cycles_.fr << ","
            << duty_cycles_.rr << "," << duty_cycles_.rl << "," << std::endl;
  log_file_.flush();
#endif

  return modified_duties;
}
}  // namespace Control