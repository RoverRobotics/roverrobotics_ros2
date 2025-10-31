#include "mecanum_robot.hpp"
namespace RoverRobotics {
MecanumRobot::MecanumRobot(const char *device,
                                     std::string new_comm,
                                     float wheel_radius,
                                     float wheel_base,
                                     float robot_length,
                                     Control::pid_gains pid,
                                     Control::angular_scaling_params angular_scale) {


  /* create object to load/store persistent parameters (ie trim) */
  persistent_params_ = std::make_unique<Utilities::PersistentParams>(ROBOT_PARAM_PATH);

  /* set comm mode: can vs serial vs other */
  if (new_comm == "serial")
    comm_type_ = "SERIAL";
  else if (new_comm == "can")
    comm_type_ = "CAN";

  /* clear main data structure for holding robot status and commands */
  robotstatus_ = {0};

  /* scaling of angular command vs linear speed; useful for teleop */
  angular_scaling_params_ = angular_scale;

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
  
  /* register the pid gains for closed-loop modes */
  pid_ = pid;

  /* make and initialize the motion logic object */
  mecanum_control__ = std::make_unique<Control::MecanumMotionController>(
      Control::INDEPENDENT_WHEEL, robot_geometry_, pid_, MOTOR_MAX_, MOTOR_MIN_,
      left_trim_, right_trim_, geometric_decay_);

  /* MUST be done after skid control is constructed */
  load_persistent_params();

  mecanum_control__->setOperatingMode(Control::INDEPENDENT_WHEEL);
  mecanum_control__->setAccelerationLimits(
          {LINEAR_JERK_LIMIT_, 30.0});
  mecanum_control__->setAngularScaling(angular_scaling_params_);


  /* set up the comm port */
  register_comm_base(device);

  /* create a dedicated write thread to send commands to the robot on fixed
   * interval */
  write_to_robot_thread_ = std::thread([this]() { this->send_command(10); });

  /* create a dedicate thread to compute the desired robot motion, runs on fixed
   * interval */
  motor_speed_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
}

void MecanumRobot::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData MecanumRobot::status_request() { 
  robotstatus_mutex_.lock();
  auto returnData = robotstatus_;
  robotstatus_mutex_.unlock();
  return returnData; 
}

robotData MecanumRobot::info_request() { 
  return status_request(); 
}

void MecanumRobot::set_robot_velocity(double *control_array) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = control_array[0];
  robotstatus_.cmd_trans_vel = control_array[3];
  robotstatus_.cmd_angular_vel = control_array[1];
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void MecanumRobot::unpack_comm_response(std::vector<uint8_t> robotmsg) {
  if (comm_type_ == "CAN") {
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
      // updating battery values for all motors including SoC
      robotstatus_.battery1_voltage = parsedMsg.voltage;
      robotstatus_.battery1_current = parsedMsg.current_in;
      if(parsedMsg.voltage >= 42.0) {
        robotstatus_.battery1_SOC = 100;
      } else if (parsedMsg.voltage <= 34.0){
        robotstatus_.battery1_SOC = 0.0;
      } else {
        robotstatus_.battery1_SOC = 12.5 * parsedMsg.voltage - 425;
      }
      robotstatus_mutex_.unlock();
    }
  } else if (comm_type_ == "SERIAL") {
    static std::vector<uint8_t> msgqueue;
    robotstatus_mutex_.lock();
    msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                    robotmsg.end());  // insert robotmsg to msg list
  
    // valid msg check
    int msg_size = msgqueue[1] + 4;
    if (msgqueue.size() >= msg_size && msgqueue[0] == START_BYTE_ &&
        msgqueue[msg_size] == STOP_BYTE_) {
      int payload_index = 3;
      int16_t v16;
      int32_t v32;
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      vesc_fet_temp_ = static_cast<double>(v16) / 10.0;
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      vesc_motor_temp_ = static_cast<double>(v16) / 10.0;
      v32 = static_cast<float>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_all_motor_current_ = static_cast<float>(v32) / 100.0;
      v32 = static_cast<float>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_all_input_current_ = static_cast<float>(v32) / 100.0;
      v32 = static_cast<int32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_id_ = static_cast<float>(v32) / 100.0;
      v32 = static_cast<int32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_iq_ = static_cast<float>(v32) / 100.0;
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      vesc_duty_ = static_cast<double>(v16) / 1000.0;
      v32 = static_cast<int32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_rpm_ = static_cast<int32_t>(v32);
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      vesc_v_in_ = static_cast<double>(v16) / 10.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_amp_hours_ = static_cast<double>(v32) / 10000.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_amp_hours_charged_ = static_cast<double>(v32) / 10000.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_watt_hours_ = static_cast<double>(v32) / 10000.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_watt_hours_charged_ = static_cast<double>(v32) / 10000.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_tach_ = static_cast<double>(v32);
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_tach_abs_ = static_cast<double>(v32);
      vesc_fault_ = static_cast<uint8_t>(msgqueue[++payload_index]);
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      vesc_pid_pos_ = static_cast<double>(v32) / 1000000.0;
      vesc_dev_id_ = static_cast<uint8_t>(msgqueue[++payload_index]);
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      double temp1 = static_cast<double>(v16) / 10.0;
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      double temp2 = static_cast<double>(v16) / 10.0;
      v16 = static_cast<int16_t>(
          (static_cast<uint16_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint16_t>(msgqueue[++payload_index]));
      double temp3 = static_cast<double>(v16) / 10.0;
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      double reset_avg_vd = static_cast<double>(v32);
      v32 = static_cast<uint32_t>(
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
          (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
          static_cast<uint32_t>(msgqueue[++payload_index]));
      double reset_avg_vq = static_cast<double>(v32);
      std::cerr << std::flush;
      msgqueue.clear();
      // msgqueue.resize(0);
      switch (vesc_dev_id_) {
          case (VESC_IDS::FRONT_LEFT):
            robotstatus_.motor1_id = vesc_dev_id_;
            robotstatus_.motor1_current = vesc_all_input_current_;
            robotstatus_.motor1_rpm = vesc_rpm_ * VESC_RPM_SCALING_FACTOR;
            robotstatus_.motor1_temp = vesc_motor_temp_;
            robotstatus_.motor1_mos_temp = vesc_fet_temp_;
            break;
          case (VESC_IDS::FRONT_RIGHT):
            robotstatus_.motor2_id = vesc_dev_id_;
            robotstatus_.motor2_current = vesc_all_input_current_;
            robotstatus_.motor2_rpm = vesc_rpm_ * VESC_RPM_SCALING_FACTOR;
            robotstatus_.motor2_temp = vesc_motor_temp_;
            robotstatus_.motor2_mos_temp = vesc_fet_temp_;
            break;
          case (VESC_IDS::BACK_LEFT):
            robotstatus_.motor3_id = vesc_dev_id_;
            robotstatus_.motor3_current = vesc_all_input_current_;
            robotstatus_.motor3_rpm = vesc_rpm_ * VESC_RPM_SCALING_FACTOR;
            robotstatus_.motor3_temp = vesc_motor_temp_;
            robotstatus_.motor3_mos_temp = vesc_fet_temp_;
            break;
          case (VESC_IDS::BACK_RIGHT):
            robotstatus_.motor4_id = vesc_dev_id_;
            robotstatus_.motor4_current = vesc_all_input_current_;
            robotstatus_.motor4_rpm = vesc_rpm_ * VESC_RPM_SCALING_FACTOR;
            robotstatus_.motor4_temp = vesc_motor_temp_;
            robotstatus_.motor4_mos_temp = vesc_fet_temp_;
            break;
          default:
            break;
        }
      robotstatus_.battery1_voltage = vesc_v_in_;
      robotstatus_.battery1_fault_flag = 0;
      robotstatus_.battery2_voltage = 0;
      robotstatus_.battery1_temp = 0;
      robotstatus_.battery2_temp = 0;
      robotstatus_.battery1_current = vesc_all_input_current_;
      robotstatus_.battery2_current = 0;
      if(robotstatus_.battery1_voltage >= 42.0) {
        robotstatus_.battery1_SOC = 100;
      } else if (robotstatus_.battery1_voltage <= 34.0){
        robotstatus_.battery1_SOC = 0.0;
      } else {
        robotstatus_.battery1_SOC = 12.5 * robotstatus_.battery1_voltage - 425;
      }
      robotstatus_.battery2_SOC = 0;
      robotstatus_.battery1_fault_flag = 0;
      robotstatus_.battery2_fault_flag = 0;
      robotstatus_.robot_guid = 0;
      robotstatus_.robot_firmware = 0;
      robotstatus_.robot_fault_flag = vesc_fault_;
      robotstatus_.robot_fan_speed = 0;
      robotstatus_.robot_speed_limit = 0;
    } else if (msgqueue.size() > msg_size && msgqueue[0] != START_BYTE_) {
      int start_byte_index = 0;
      // !Did not find valid start byte in buffer
      while (msgqueue[start_byte_index] != START_BYTE_ &&
             start_byte_index < msgqueue.size())
        start_byte_index++;
      if (start_byte_index >= msgqueue.size()) {
        msgqueue.clear();
        return;
      } else {
        // !Reconstruct the vector so that the start byte is at the 0 position
        std::vector<uint8_t> temp;
        for (int x = start_byte_index; x < msgqueue.size(); x++) {
          temp.push_back(msgqueue[x]);
        }
        msgqueue.clear();
        msgqueue.resize(0);
        msgqueue = temp;
        temp.clear();
      }
    }
    robotstatus_mutex_.unlock();
  }
}

bool MecanumRobot::is_connected() { return comm_base_->is_connected(); }

void MecanumRobot::register_comm_base(const char *device) {
  std::vector<uint8_t> setting;
  if (comm_type_ == "CAN") {
    try {
      // if(strcmp(device, "internal") != 0) {
        comm_base_ = std::make_unique<CommCan>(
            device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
            setting);
      // } else {
      //   /*
      //   comm_base_ = std::make_unique<CommCanSPI>(
      //       device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
      //       setting);
      //   */
      // }  
    } catch (int i) {
      throw(i);
    }
  } else {
    throw(-2);
  }
}

void MecanumRobot::send_command(int sleeptime) {
  while (true) {

    /* loop over the motors */
    for (uint8_t vid = VESC_IDS::FRONT_LEFT; vid <= VESC_IDS::BACK_RIGHT;
         vid++) {

      robotstatus_mutex_.lock();
      float signedMotorCommand = motors_speeds_[vid];

      /* only use current control when robot is stopped to prevent wasted energy
       */
      bool useCurrentControl = motors_speeds_[vid] == MOTOR_NEUTRAL_ &&
                               robotstatus_.linear_vel == MOTOR_NEUTRAL_ &&
                               robotstatus_.trans_vel == MOTOR_NEUTRAL_ &&
                               robotstatus_.angular_vel == MOTOR_NEUTRAL_;

      robotstatus_mutex_.unlock();

      auto msg =  vescArray_.buildCommandMessage((vesc::vescChannelCommand){
              .vescId = vid,
              .commandType = (useCurrentControl ? vesc::vescPacketFlags::CURRENT
                                                : vesc::vescPacketFlags::DUTY),
              .commandValue = (useCurrentControl ? MOTOR_NEUTRAL_ : signedMotorCommand)});

      comm_base_->write_to_device(msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

int MecanumRobot::cycle_robot_mode() {
    // ONLY CLOSED LOOP CONTROL FOR 4WD
    return -1;
}

void MecanumRobot::load_persistent_params() {
  
  /* trim (aka curvature correction) */
  if(auto param = persistent_params_->read_param("trim")){
    update_drivetrim(param.value());
    std::cout << "Loaded trim from persistent param file: " << param.value() << std::endl;
  }
}

void MecanumRobot::update_drivetrim(double delta) {

  if (-MAX_CURVATURE_CORRECTION_ < (trimvalue_ + delta) && (trimvalue_ + delta) < MAX_CURVATURE_CORRECTION_) {
    trimvalue_ += delta;

    /* reduce power to right wheels */
    if (trimvalue_ >= 0) {
      left_trim_ = 1;
      right_trim_ = 1 - trimvalue_;
    }
    /* reduce power to left wheels */
    else {
      right_trim_ = 1;
      left_trim_ = 1 + trimvalue_;
    }
    mecanum_control__->setTrim(left_trim_, right_trim_);
    std::cout << "writing trim " << trimvalue_ << " to file " << std::endl;
    persistent_params_->write_param("trim", trimvalue_);
  }
  
}

void MecanumRobot::motors_control_loop(int sleeptime) {
  float linear_vel_target, trans_vel_target, angular_vel_target, rpm_FL, rpm_FR, rpm_BL, rpm_BR;
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
    trans_vel_target = robotstatus_.cmd_trans_vel;
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
      auto wheel_speeds = mecanum_control__->runMotionControl(
          (Control::robot_velocities){.linear_velocity = linear_vel_target,
                                      .trans_velocity = trans_vel_target,
                                      .angular_velocity = angular_vel_target},
          (Control::motor_data){.fl = 0, .fr = 0, .rl = 0, .rr = 0},
          (Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR});

      
      
      /* compute velocities of robot from wheel rpms */
      auto velocities = mecanum_control__->getMeasuredVelocities((Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR});

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[VESC_IDS::FRONT_LEFT] = wheel_speeds.fl;
      motors_speeds_[VESC_IDS::FRONT_RIGHT] = wheel_speeds.fr;
      motors_speeds_[VESC_IDS::BACK_LEFT] = wheel_speeds.rl;
      motors_speeds_[VESC_IDS::BACK_RIGHT] = wheel_speeds.rr;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.trans_vel = velocities.trans_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();

    } else {

      /* COMMAND THE ROBOT TO STOP */
      auto wheel_speeds = mecanum_control__->runMotionControl(
          {0, 0, 0}, {0, 0, 0, 0}, {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      auto velocities = mecanum_control__->getMeasuredVelocities(
          {rpm_FL, rpm_FR, rpm_BL, rpm_BR});

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[VESC_IDS::FRONT_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::FRONT_RIGHT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::BACK_LEFT] = MOTOR_NEUTRAL_;
      motors_speeds_[VESC_IDS::BACK_RIGHT] = MOTOR_NEUTRAL_;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.trans_vel = velocities.trans_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

}  // namespace RoverRobotics
