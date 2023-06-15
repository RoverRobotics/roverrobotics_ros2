#include "differential_robot.hpp"
namespace RoverRobotics {
DifferentialRobot::DifferentialRobot(const char *device, 
                                     float wheel_radius,
                                     float wheel_base,
                                     float robot_length) {

  /* set comm mode: can vs serial vs other */
  comm_type_ = "CAN";

  /* clear main data structure for holding robot status and commands */
  robotstatus_ = {0};

  /* Set robot geometry */
  robot_geometry_ = {.intra_axle_distance = robot_length,
                     .wheel_base = wheel_base,
                     .wheel_radius = wheel_radius,
                     .center_of_mass_x_offset = 0,
                     .center_of_mass_y_offset = 0};

  /* clear estop and zero out all motors */
  estop_ = false;
  motors_speeds_[VESC_IDS::FRONT_LEFT] = MOTOR_NEUTRAL_;
  motors_speeds_[VESC_IDS::FRONT_RIGHT] = MOTOR_NEUTRAL_;
  motors_speeds_[VESC_IDS::BACK_LEFT] = MOTOR_NEUTRAL_;
  motors_speeds_[VESC_IDS::BACK_RIGHT] = MOTOR_NEUTRAL_;

  /* make an object to decode and encode motor controller messages*/
  vescArray_ = vesc::BridgedVescArray(
      std::vector<uint8_t>{VESC_IDS::FRONT_LEFT, VESC_IDS::FRONT_RIGHT,
                           VESC_IDS::BACK_LEFT, VESC_IDS::BACK_RIGHT});

  /* set up the comm port */
  register_comm_base(device);

  /* create a dedicated write thread to send commands to the robot on fixed
   * interval */
  write_to_robot_thread_ = std::thread([this]() { this->send_command(30); });

  /* create a dedicate thread to compute the desired robot motion, runs on fixed
   * interval */
  motor_speed_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
}

void DifferentialRobot::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData DifferentialRobot::status_request() { 
  robotstatus_mutex_.lock();
  auto returnData = robotstatus_;
  robotstatus_mutex_.unlock();
  return returnData; 
}

robotData DifferentialRobot::info_request() { 
  return status_request(); 
}

void DifferentialRobot::set_robot_velocity(double *control_array) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = control_array[0];
  robotstatus_.cmd_angular_vel = control_array[1];
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void DifferentialRobot::unpack_comm_response(std::vector<uint8_t> robotmsg) {
  auto parsedMsg = vescArray_.parseReceivedMessage(robotmsg);
  if (parsedMsg.dataValid) {
    robotstatus_mutex_.lock();
    switch (parsedMsg.vescId) {
      case (VESC_IDS::FRONT_LEFT):
        robotstatus_.motor1_rpm = parsedMsg.rpm;
        robotstatus_.motor1_id = parsedMsg.vescId;
        robotstatus_.motor1_current = parsedMsg.current;
        break;
      case (VESC_IDS::FRONT_RIGHT):
        robotstatus_.motor2_rpm = parsedMsg.rpm;
        robotstatus_.motor2_id = parsedMsg.vescId;
        robotstatus_.motor2_current = parsedMsg.current;
        break;
      case (VESC_IDS::BACK_LEFT):
        robotstatus_.motor3_rpm = parsedMsg.rpm;
        robotstatus_.motor3_id = parsedMsg.vescId;
        robotstatus_.motor3_current = parsedMsg.current;
        break;
      case (VESC_IDS::BACK_RIGHT):
        robotstatus_.motor4_rpm = parsedMsg.rpm;
        robotstatus_.motor4_id = parsedMsg.vescId;
        robotstatus_.motor4_current = parsedMsg.current;
        break;
      default:
        break;
    }
    robotstatus_mutex_.unlock();
  }
}

bool DifferentialRobot::is_connected() { return comm_base_->is_connected(); }

void DifferentialRobot::register_comm_base(const char *device) {
  std::vector<uint8_t> setting;
  if (comm_type_ == "CAN") {
    try {
      comm_base_ = std::make_unique<CommCan>(
          device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      throw(i);
    }
  } else if (comm_type_ == "UART") {
    try {
      comm_base_ = std::make_unique<CommSerial>(
          device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      throw(i);
    }
  } else {
    
  }
}

void DifferentialRobot::send_command(int sleeptime) {
  while (true) {

    /* loop over the motors */
    for (uint8_t vid = VESC_IDS::FRONT_LEFT; vid <= VESC_IDS::BACK_RIGHT;
         vid++) {

      robotstatus_mutex_.lock();
      auto signedMotorCommand = motors_speeds_[vid];

      /* only use current control when robot is stopped to prevent wasted energy
       */
      bool useCurrentControl = motors_speeds_[vid] == MOTOR_NEUTRAL_ &&
                               robotstatus_.linear_vel == MOTOR_NEUTRAL_ &&
                               robotstatus_.angular_vel == MOTOR_NEUTRAL_;

      robotstatus_mutex_.unlock();

      auto msg =  vescArray_.buildCommandMessage((vesc::vescChannelCommand){
              .vescId = vid,
              .commandType = (useCurrentControl ? vesc::vescPacketFlags::CURRENT
                                                : vesc::vescPacketFlags::SETRPM),
              .commandValue = (useCurrentControl ? (float)MOTOR_NEUTRAL_ : signedMotorCommand)});

      comm_base_->write_to_device(msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}


void DifferentialRobot::motors_control_loop(int sleeptime) {
  float linear_vel_target, angular_vel_target, rpm_FL, rpm_FR, rpm_BL, rpm_BR;
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());

    /* collect user commands and various status */
    robotstatus_mutex_.lock();
    linear_vel_target = robotstatus_.cmd_linear_vel;
    angular_vel_target = robotstatus_.cmd_angular_vel;
    rpm_FL = robotstatus_.motor1_rpm;
    rpm_FR = robotstatus_.motor2_rpm;
    rpm_BL = robotstatus_.motor3_rpm;
    rpm_BR = robotstatus_.motor4_rpm;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();

    /* compute motion targets if no estop and data is not stale */
    if (!estop_ &&
        (time_now - time_from_msg).count() <= CONTROL_LOOP_TIMEOUT_MS_) {
      
      /* compute motion targets (not using duty cycle input ATM) */
      auto wheel_speeds = computeDifferentialWheelSpeeds(
        (Control::robot_velocities){linear_vel_target, angular_vel_target}, robot_geometry_);

      
      
      /* compute velocities of robot from wheel rpms */
      auto velocities = computeVelocitiesFromWheelspeeds((Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR}, robot_geometry_);

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[VESC_IDS::FRONT_LEFT] = wheel_speeds.fl;
      motors_speeds_[VESC_IDS::FRONT_RIGHT] = wheel_speeds.fr;
      motors_speeds_[VESC_IDS::BACK_LEFT] = wheel_speeds.rl;
      motors_speeds_[VESC_IDS::BACK_RIGHT] = wheel_speeds.rr;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();

    } else {

      /* COMMAND THE ROBOT TO STOP */
      auto wheel_speeds = computeDifferentialWheelSpeeds(
        (Control::robot_velocities){0.0, 0.0}, robot_geometry_);
      auto velocities = computeVelocitiesFromWheelspeeds((Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR}, robot_geometry_);

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[VESC_IDS::FRONT_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::FRONT_RIGHT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::BACK_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::BACK_RIGHT] = MOTOR_NEUTRAL_;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

Control::motor_data DifferentialRobot::computeDifferentialWheelSpeeds(Control::robot_velocities target_velocities,
                                       Control::robot_geometry robot_geometry) {
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

  Control::motor_data returnstruct = {left_wheel_speed, right_wheel_speed,
                             left_wheel_speed, right_wheel_speed};
  return returnstruct;
}

Control::robot_velocities DifferentialRobot::computeVelocitiesFromWheelspeeds(
    Control::motor_data wheel_speeds, Control::robot_geometry robot_geometry) {
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

  Control::robot_velocities returnstruct;
  returnstruct.linear_velocity = linear_velocity;
  returnstruct.angular_velocity = angular_velocity;
  return returnstruct;
}

}  // namespace RoverRobotics
