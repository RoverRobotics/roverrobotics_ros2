#include "protocol_zero_2.hpp"

namespace RoverRobotics {

Zero2ProtocolObject::Zero2ProtocolObject(
    const char *device, std::string new_comm_type,
    Control::robot_motion_mode_t robot_mode, Control::pid_gains pid,
    Control::angular_scaling_params angular_scale) {
  /* create object to load/store persistent parameters (ie trim) */
  persistent_params_ =
      std::make_unique<Utilities::PersistentParams>(ROBOT_PARAM_PATH);
  /* set comm mode: can vs serial vs other */
  comm_type_ = new_comm_type;
  /* set drive mode: open loop, traction control, independent wheel */
  robot_mode_ = robot_mode;
  /* scaling of angular command vs linear speed; useful for teleop */
  angular_scaling_params_ = angular_scale;
  /* clear main data structure for holding robot status and commands */
  robotstatus_ = {0};
  /* clear estop and zero out all motors */
  estop_ = false;
  motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
  /* register the pid gains for closed-loop modes */
  pid_ = pid;

  /* make and initialize the motion logic object */
  skid_control_ = std::make_unique<Control::SkidRobotMotionController>(
      Control::OPEN_LOOP, robot_geometry_, pid_, MOTOR_MAX_, MOTOR_MIN_,
      left_trim_, right_trim_, geometric_decay_);

  /* MUST be done after skid control is constructed */
  load_persistent_params();

  /* set some default params */
  skid_control_->setOpenLoopMaxRpm(OPEN_LOOP_MAX_RPM_);
  skid_control_->setAngularScaling(angular_scaling_params_);

  /* set mode specific limits */
  if (robot_mode_ != Control::OPEN_LOOP) {
    closed_loop_ = true;
    skid_control_->setOperatingMode(Control::INDEPENDENT_WHEEL);
    skid_control_->setAccelerationLimits(
        {LINEAR_JERK_LIMIT_, std::numeric_limits<float>::max()});
    robotmode_num_ = 1;
  } else {
    closed_loop_ = false;
    skid_control_->setOperatingMode(Control::OPEN_LOOP);
    skid_control_->setAccelerationLimits(
        {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()});
    robotmode_num_ = 0;
  }
    
  /* set up the comm port */
    
  /*
  
  std::cerr << "establishing connection to rover zero..." << std::endl;
  try{
  
  */
    
  register_comm_base(device);
    
    /*
  }
  catch{
      std::cerr << "error establishing connection to Rover Zero, please check cabling and power to the motor controller (VESC)" << std::endl;
  }
  
  */
    
    
  /* create a dedicated write thread to send commands to the robot on fixed
   * interval */
 /* std::cerr << "creating thread to communicate with rover zero..." << std::endl; */
  write_to_robot_thread_ =
  
      std::thread([this]() { this->send_getvalues_command(10); });
    
  /* create a dedicate thread to compute the desired robot motion, runs on fixed
   * interval */
  motor_speed_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
  std::cerr << "protocol is running..." << std::endl;
}

void Zero2ProtocolObject::load_persistent_params() {
  /* trim (aka curvature correction) */
  if (auto param = persistent_params_->read_param("trim")) {
    update_drivetrim(param.value());
    std::cout << "Loaded trim from persistent param file: " << param.value()
              << std::endl;
  }
}

void Zero2ProtocolObject::update_drivetrim(double delta) {
  if (-MAX_CURVATURE_CORRECTION_ < (trimvalue_ + delta) &&
      (trimvalue_ + delta) < MAX_CURVATURE_CORRECTION_) {
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
    skid_control_->setTrim(left_trim_, right_trim_);
    std::cout << "writing trim " << trimvalue_ << " to file " << std::endl;
    persistent_params_->write_param("trim", trimvalue_);
  }
}

void Zero2ProtocolObject::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData Zero2ProtocolObject::status_request() { return robotstatus_; }

robotData Zero2ProtocolObject::info_request() { return robotstatus_; }

void Zero2ProtocolObject::set_robot_velocity(double *controlarray) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = controlarray[0];
  robotstatus_.cmd_angular_vel = controlarray[1];
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void Zero2ProtocolObject::motors_control_loop(int sleeptime) {
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
    /* Convert from motors to wheels RPM based on the robot geometry and gear
     * ratio */
    rpm_FL = robotstatus_.motor1_rpm / MOTOR_RPM_TO_WHEEL_RPM_RATIO_;
    rpm_FR = robotstatus_.motor2_rpm / MOTOR_RPM_TO_WHEEL_RPM_RATIO_;
    rpm_BL = robotstatus_.motor1_rpm / MOTOR_RPM_TO_WHEEL_RPM_RATIO_;
    rpm_BR = robotstatus_.motor2_rpm / MOTOR_RPM_TO_WHEEL_RPM_RATIO_;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();

    /* compute motion targets if no estop and data is not stale */
    if (!estop_ &&
        (time_now - time_from_msg).count() <= CONTROL_LOOP_TIMEOUT_MS_) {
      /* compute motion targets (not using duty cycle input ATM) */
      auto duty_cycles = skid_control_->runMotionControl(
          (Control::robot_velocities){.linear_velocity = linear_vel_target,
                                      .angular_velocity = angular_vel_target},
          (Control::motor_data){.fl = 0, .fr = 0, .rl = 0, .rr = 0},
          (Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR});

      /* compute velocities of robot from wheel rpms */
      auto velocities =
          skid_control_->getMeasuredVelocities((Control::motor_data){
              .fl = rpm_FL, .fr = rpm_FR, .rl = rpm_BL, .rr = rpm_BR});

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[LEFT_MOTOR] = duty_cycles.fl;
      motors_speeds_[RIGHT_MOTOR] = duty_cycles.fr;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
      send_motors_commands();
    } else {
      /* COMMAND THE ROBOT TO STOP */
      auto duty_cycles = skid_control_->runMotionControl(
          {0, 0}, {0, 0, 0, 0}, {rpm_FL, rpm_FR, rpm_BL, rpm_BR});
      auto velocities = skid_control_->getMeasuredVelocities(
          {rpm_FL, rpm_FR, rpm_BL, rpm_BR});

      /* update the main data structure with both commands and status */
      robotstatus_mutex_.lock();
      motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
      motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
      robotstatus_.linear_vel = velocities.linear_velocity;
      robotstatus_.angular_vel = velocities.angular_velocity;
      robotstatus_mutex_.unlock();
      send_motors_commands();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}
void Zero2ProtocolObject::unpack_comm_response(std::vector<uint8_t> robotmsg) {
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
    v32 = static_cast<int32_t>(
        (static_cast<uint32_t>(msgqueue[++payload_index]) << 24) +
        (static_cast<uint32_t>(msgqueue[++payload_index]) << 16) +
        (static_cast<uint32_t>(msgqueue[++payload_index]) << 8) +
        static_cast<uint32_t>(msgqueue[++payload_index]));
    vesc_all_motor_current_ = static_cast<float>(v32) / 100.0;
    v32 = static_cast<int32_t>(
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
    if (vesc_dev_id_ == LEFT_MOTOR) {
      robotstatus_.motor1_id = vesc_dev_id_;
      robotstatus_.motor1_current = vesc_all_input_current_;
      robotstatus_.motor1_rpm = vesc_rpm_;
      robotstatus_.motor1_temp = vesc_motor_temp_;
      robotstatus_.motor1_mos_temp = vesc_fet_temp_;
    } else if (vesc_dev_id_ == RIGHT_MOTOR) {
      robotstatus_.motor2_id = vesc_dev_id_;
      robotstatus_.motor2_current = vesc_all_input_current_;
      robotstatus_.motor2_rpm = vesc_rpm_;
      robotstatus_.motor2_temp = vesc_motor_temp_;
      robotstatus_.motor2_mos_temp = vesc_fet_temp_;
    }
    robotstatus_.battery1_voltage = vesc_v_in_;
    robotstatus_.battery1_fault_flag = 0;
    robotstatus_.battery2_voltage = 0;
    robotstatus_.battery1_temp = 0;
    robotstatus_.battery2_temp = 0;
    robotstatus_.battery1_current = vesc_all_input_current_;
    robotstatus_.battery2_current = 0;
    robotstatus_.battery1_SOC = 0;
    if(robotstatus_.battery1_voltage >= 16.5) {
      robotstatus_.battery1_SOC = 100;
    } else if (robotstatus_.battery1_voltage <= 13.5){
      robotstatus_.battery1_SOC = 0.0;
    } else {
      robotstatus_.battery1_SOC = 33.3333 * robotstatus_.battery1_voltage - 450;
    }
    robotstatus_.battery2_SOC = 0;
    robotstatus_.battery1_fault_flag = 0;
    robotstatus_.battery2_fault_flag = 0;
    robotstatus_.motor3_rpm = 0;
    robotstatus_.motor3_current = 0;
    robotstatus_.motor3_temp = 0;
    robotstatus_.motor3_mos_temp = 0;
    robotstatus_.motor4_id = 0;
    robotstatus_.motor4_rpm = 0;
    robotstatus_.motor4_current = 0;
    robotstatus_.motor4_temp = 0;
    robotstatus_.motor4_mos_temp = 0;
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

bool Zero2ProtocolObject::is_connected() { return comm_base_->is_connected(); }

int Zero2ProtocolObject::cycle_robot_mode() {
  robotmode_num_ = ++robotmode_num_ % (ROBOT_MODES_);

  switch (robotmode_num_) {
    case Control::OPEN_LOOP:
      skid_control_->setOperatingMode(Control::OPEN_LOOP);
      skid_control_->setAccelerationLimits({std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max()});
      break;
    case Control::TRACTION_CONTROL:
      skid_control_->setOperatingMode(Control::INDEPENDENT_WHEEL);
      skid_control_->setAccelerationLimits(
          {LINEAR_JERK_LIMIT_, std::numeric_limits<float>::max()});
      break;
  }
  std::cout << "robot_mode: " << robotmode_num_ << std::endl;
  return robotmode_num_;
}
void Zero2ProtocolObject::register_comm_base(const char *device) {
  if (comm_type_ == "serial") {
    std::vector<uint8_t> setting;
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 24));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 16));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 8));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_));
    setting.push_back(RECEIVE_MSG_LEN_);
    try {
      comm_base_ = std::make_unique<CommSerial>(
          device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
      std::cerr << "error";
      throw(i);
    }
  } else {  // not supported device
    std::cerr << "not supported";
    throw(-2);
  }
}

void Zero2ProtocolObject::send_getvalues_command(int sleeptime) {
  while (true) {
    if (comm_type_ == "serial") {
      unsigned char *payloadptr;
      uint16_t crc;
      std::vector<uint8_t> write_buffer;
      uint8_t MSG_SIZE = 1;
      robotstatus_mutex_.lock();
      uint8_t payload[1];
      payload[0] = COMM_GET_VALUES;
      payloadptr = payload;
      write_buffer = {PAYLOAD_BYTE_SIZE_, MSG_SIZE, COMM_GET_VALUES};
      crc = crc16(payloadptr, MSG_SIZE);
      write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
      write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
      write_buffer.push_back(STOP_BYTE_);
      comm_base_->write_to_device(write_buffer);
      robotstatus_mutex_.unlock();

      robotstatus_mutex_.lock();
      uint8_t payload2[3];
      payload2[0] = COMM_CAN_FORWARD;
      payload2[1] = RIGHT_MOTOR;
      payload2[2] = COMM_GET_VALUES;
      payloadptr = payload2;
      MSG_SIZE = 3;
      write_buffer.clear();
      write_buffer = {PAYLOAD_BYTE_SIZE_, MSG_SIZE, COMM_CAN_FORWARD,
                      RIGHT_MOTOR, COMM_GET_VALUES};
      crc = crc16(payloadptr, MSG_SIZE);
      write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
      write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
      write_buffer.push_back(STOP_BYTE_);
      comm_base_->write_to_device(write_buffer);
      robotstatus_mutex_.unlock();
    } else if (comm_type_ == "can") {
      return;
    } else {   //! How did you get here?
      return;  // TODO: Return error ?
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
  }
}

void Zero2ProtocolObject::send_motors_commands() {
  robotstatus_mutex_.lock();
  int32_t v = static_cast<int32_t>(motors_speeds_[LEFT_MOTOR] * 100000.0);
  unsigned char *payloadptr;
  uint8_t payload[5];
  payload[0] = COMM_SET_DUTY;
  payload[1] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  payload[2] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  payload[3] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  payload[4] = static_cast<uint8_t>((static_cast<uint32_t>(v)) & 0xFF);
  payloadptr = payload;
  std::vector<uint8_t> write_buffer = {
      PAYLOAD_BYTE_SIZE_,
      MSG_SIZE_,
      COMM_SET_DUTY,
      static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
      static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
      static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
      static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF)};

  uint16_t crc = crc16(payloadptr, write_buffer[1]);
  write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
  write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
  write_buffer.push_back(STOP_BYTE_);
  comm_base_->write_to_device(write_buffer);
  robotstatus_mutex_.unlock();
  write_buffer.clear();
  robotstatus_mutex_.lock();
  // WIP
  v = static_cast<int32_t>(motors_speeds_[RIGHT_MOTOR] * 100000.0);
  unsigned char payload2[7];
  payload2[0] = COMM_CAN_FORWARD;
  payload2[1] = RIGHT_MOTOR;
  payload2[2] = COMM_SET_DUTY;
  payload2[3] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  payload2[4] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  payload2[5] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  payload2[6] = static_cast<uint8_t>((static_cast<uint32_t>(v)) & 0xFF);
  payloadptr = payload2;
  write_buffer = {PAYLOAD_BYTE_SIZE_,
                  FORWARD_MSG_SIZE_,
                  COMM_CAN_FORWARD,
                  RIGHT_MOTOR,
                  COMM_SET_DUTY,
                  static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF),
                  static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF),
                  static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF),
                  static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF)};
  crc = crc16(payload2, FORWARD_MSG_SIZE_);
  write_buffer.push_back(static_cast<uint8_t>(crc >> 8));
  write_buffer.push_back(static_cast<uint8_t>(crc & 0xFF));
  write_buffer.push_back(STOP_BYTE_);
  comm_base_->write_to_device(write_buffer);
  robotstatus_mutex_.unlock();
}
unsigned short Zero2ProtocolObject::crc16(unsigned char *buf,
                                          unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}
}  // namespace RoverRobotics
