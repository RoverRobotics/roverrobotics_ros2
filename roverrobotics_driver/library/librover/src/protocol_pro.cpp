#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char *device,
                                     std::string new_comm_type,
                                     Control::robot_motion_mode_t robot_mode,
                                     Control::pid_gains pid) {
  comm_type_ = new_comm_type;
  robot_mode_ = robot_mode;
  robotstatus_ = {0};
  estop_ = false;
  motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL_;
  std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
                                     REG_MOTOR_FB_RPM_RIGHT, EncoderInterval_0,
                                     EncoderInterval_1};
  std::vector<uint32_t> slow_data = {
      REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
      REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
      REG_MOTOR_CHARGER_STATE,   BuildNO,
      BATTERY_VOLTAGE_A,         REG_PWR_BAT_VOLTAGE_A};
  pid_ = pid;
  PidGains oldgain = {pid_.kp, pid_.ki, pid_.kd};
  if (robot_mode_ != Control::OPEN_LOOP)
    closed_loop_ = true;
  else
    closed_loop_ = false;
  motor1_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);
  motor2_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);

  register_comm_base(device);

  // Create a New Thread with 30 mili seconds sleep timer
  fast_data_write_thread_ =
      std::thread([this, fast_data]() { this->send_command(30, fast_data); });
  // Create a new Thread with 50 mili seconds sleep timer
  slow_data_write_thread_ =
      std::thread([this, slow_data]() { this->send_command(50, slow_data); });
  // Create a motor update thread with 30 mili second sleep timer
  motor_commands_update_thread_ =
      std::thread([this]() { this->motors_control_loop(30); });
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue_ += value; }

void ProProtocolObject::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  estop_ = estop;
  robotstatus_mutex_.unlock();
}

robotData ProProtocolObject::status_request() {
  return robotstatus_;
}

robotData ProProtocolObject::info_request() { return robotstatus_; }

void ProProtocolObject::set_robot_velocity(double *controlarray) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_linear_vel = controlarray[0];
  robotstatus_.cmd_angular_vel = controlarray[1];
  motors_speeds_[FLIPPER_MOTOR] =
      (int)round(controlarray[2] + MOTOR_NEUTRAL_) % MOTOR_MAX_;
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void ProProtocolObject::motors_control_loop(int sleeptime) {
  double linear_vel;
  double angular_vel;
  double rpm1;
  double rpm2;

  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    robotstatus_mutex_.lock();
    int firmware = robotstatus_.robot_firmware;
    linear_vel = robotstatus_.cmd_linear_vel;
    angular_vel = robotstatus_.cmd_angular_vel;
    rpm1 = robotstatus_.motor1_rpm;
    rpm2 = robotstatus_.motor2_rpm;
    time_from_msg = robotstatus_.cmd_ts;
    robotstatus_mutex_.unlock();
    float ctrl_update_elapsedtime = (time_now - time_from_msg).count();
    float pid_update_elapsedtime = (time_now - time_last).count();

    if (ctrl_update_elapsedtime > CONTROL_LOOP_TIMEOUT_MS_ || estop_) {
      robotstatus_mutex_.lock();
      motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
      motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
      motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL_;
      motor1_control_.reset();
      motor2_control_.reset();
      robotstatus_mutex_.unlock();
      time_last = time_now;
      continue;
    }

    if (angular_vel == 0) {
      if (linear_vel > 0) {
        angular_vel = trimvalue_;
      } else if (linear_vel < 0) {
        angular_vel = -trimvalue_;
      }
    }
    // !Applying some Skid-steer math
    double motor1_vel = linear_vel - 0.5 * wheel2wheelDistance * angular_vel;
    double motor2_vel = linear_vel + 0.5 * wheel2wheelDistance * angular_vel;
    if (motor1_vel == 0) motor1_control_.reset();
    if (motor2_vel == 0) motor2_control_.reset();
    if (firmware == OVF_FIXED_FIRM_VER_) {  // check firmware version
      rpm1 = rpm1 * 2;
      rpm2 = rpm2 * 2;
    }
    double motor1_measured_vel = rpm1 / MOTOR_RPM_TO_MPS_RATIO_;
    double motor2_measured_vel = rpm2 / MOTOR_RPM_TO_MPS_RATIO_;
    robotstatus_mutex_.lock();
    // motor speeds in m/s
    motors_speeds_[LEFT_MOTOR] =
        motor1_control_.run(motor1_vel, motor1_measured_vel,
                            pid_update_elapsedtime / 1000, firmware);
    motors_speeds_[RIGHT_MOTOR] =
        motor2_control_.run(motor2_vel, motor2_measured_vel,
                            pid_update_elapsedtime / 1000, firmware);

    // Convert to 8 bit Command
    motors_speeds_[LEFT_MOTOR] = motor1_control_.boundMotorSpeed(
        int(round(motors_speeds_[LEFT_MOTOR] * 50 + MOTOR_NEUTRAL_)),
        MOTOR_MAX_, MOTOR_MIN_);

    motors_speeds_[RIGHT_MOTOR] = motor2_control_.boundMotorSpeed(
        int(round(motors_speeds_[RIGHT_MOTOR] * 50 + MOTOR_NEUTRAL_)),
        MOTOR_MAX_, MOTOR_MIN_);
    robotstatus_mutex_.unlock();
    time_last = time_now;
  }
}
void ProProtocolObject::unpack_comm_response(std::vector<uint8_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  robotstatus_mutex_.lock();
  msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                  robotmsg.end());  // insert robotmsg to msg list
  // ! Delete bytes until valid start byte is found
  if ((unsigned char)msgqueue[0] != startbyte_ &&
      msgqueue.size() > RECEIVE_MSG_LEN_) {
    int startbyte_index = 0;
    // !Did not find valid start byte in buffer
    while (msgqueue[startbyte_index] != startbyte_ &&
           startbyte_index < msgqueue.size())
      startbyte_index++;
    if (startbyte_index >= msgqueue.size()) {
      msgqueue.clear();
      return;
    } else {
      // !Reconstruct the vector so that the start byte is at the 0 position
      std::vector<uint32_t> temp;
      for (int x = startbyte_index; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }
  }
  if ((unsigned char)msgqueue[0] == startbyte_ &&
      msgqueue.size() >= RECEIVE_MSG_LEN_) {  // if valid start byte
    unsigned char start_byte_read, data1, data2, dataNO, checksum,
        read_checksum;
    start_byte_read = (unsigned char)msgqueue[0];
    dataNO = (unsigned char)msgqueue[1];
    data1 = (unsigned char)msgqueue[2];
    data2 = (unsigned char)msgqueue[3];
    checksum = 255 - (dataNO + data1 + data2) % 255;
    read_checksum = (unsigned char)msgqueue[4];
    if (checksum == read_checksum) {  // verify checksum
      int16_t b = (data1 << 8) + data2;
      switch (int(dataNO)) {
        case REG_PWR_TOTAL_CURRENT:
          break;
        case REG_MOTOR_FB_RPM_LEFT:
          robotstatus_.motor1_rpm = b;
          break;
        case REG_MOTOR_FB_RPM_RIGHT:  // motor2_rpm;
          robotstatus_.motor2_rpm = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT1:
          robotstatus_.motor3_sensor1 = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT2:
          robotstatus_.motor3_sensor2 = b;
          break;
        case REG_MOTOR_FB_CURRENT_LEFT:
          robotstatus_.motor1_current = b;
          break;
        case REG_MOTOR_FB_CURRENT_RIGHT:
          robotstatus_.motor2_current = b;
          break;
        case REG_MOTOR_ENCODER_COUNT_LEFT:
          break;
        case REG_MOTOR_ENCODER_COUNT_RIGHT:
          break;
        case REG_MOTOR_FAULT_FLAG_LEFT:
          robotstatus_.robot_fault_flag = b;
          break;
        case REG_MOTOR_TEMP_LEFT:
          robotstatus_.motor1_temp = b;
          break;
        case REG_MOTOR_TEMP_RIGHT:
          robotstatus_.motor2_temp = b;
          break;
        case REG_PWR_BAT_VOLTAGE_A:
          if (robotstatus_.robot_firmware == 10009) {
            robotstatus_.battery1_SOC = b;
          }
          break;
        case REG_PWR_BAT_VOLTAGE_B:
          break;
        case EncoderInterval_0:
          break;
        case EncoderInterval_1:
          break;
        case EncoderInterval_2:
          break;
        case REG_ROBOT_REL_SOC_A:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery1_SOC = b;
          }
          break;
        case REG_ROBOT_REL_SOC_B:
          break;
        case REG_MOTOR_CHARGER_STATE:
          break;
        case BuildNO:
          robotstatus_.robot_firmware = b;
          break;
        case REG_PWR_A_CURRENT:
          break;
        case REG_PWR_B_CURRENT:
          break;
        case REG_MOTOR_FLIPPER_ANGLE:
          robotstatus_.motor3_angle = b;
          break;
        case to_computer_REG_MOTOR_SIDE_FAN_SPEED:
          robotstatus_.robot_fan_speed = b;
          break;
        case to_computer_REG_MOTOR_SLOW_SPEED:
          break;
        case BATTERY_STATUS_A:
          break;
        case BATTERY_STATUS_B:
          break;
        case BATTERY_MODE_A:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery1_fault_flag = b;
          }
          break;
        case BATTERY_MODE_B:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery2_fault_flag = b;
          }
          break;
        case BATTERY_TEMP_A:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery1_temp = b;
          }
          break;
        case BATTERY_TEMP_B:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery2_temp = b;
          }
          break;
        case BATTERY_VOLTAGE_A:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery1_voltage = b;
          }
          break;
        case BATTERY_VOLTAGE_B:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery2_voltage = b;
          }
          break;
        case BATTERY_CURRENT_A:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery1_current = b;
          }
          break;
        case BATTERY_CURRENT_B:
          if (robotstatus_.robot_firmware != OVF_FIXED_FIRM_VER_) {
            robotstatus_.battery2_current = b;
          }
          break;
      }
      // !Same battery system for both A and B on this robot
      robotstatus_.battery2_SOC = robotstatus_.battery1_SOC;
      // !THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
      robotstatus_.motor1_id = 0;
      robotstatus_.motor1_mos_temp = 0;
      robotstatus_.motor2_id = 0;
      robotstatus_.motor2_mos_temp = 0;
      robotstatus_.motor3_id = 0;
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
      robotstatus_.robot_speed_limit = 0;
      if (robotstatus_.robot_firmware == OVF_FIXED_FIRM_VER_) {  // check firmware version
        robotstatus_.linear_vel =
            0.5 * (robotstatus_.motor1_rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_ +
                   robotstatus_.motor2_rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_);

        robotstatus_.angular_vel =
            ((robotstatus_.motor2_rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_) -
             (robotstatus_.motor1_rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_)) *
            odom_angular_coef_ * odom_traction_factor_;
      } else {
        robotstatus_.linear_vel =
            0.5 * (robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_ +
                   robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_);

        robotstatus_.angular_vel =
            ((robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO_) -
             (robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO_)) *
            odom_angular_coef_ * odom_traction_factor_;
      }

      std::vector<uint32_t> temp;
      // !Remove processed msg from queue
      for (int x = RECEIVE_MSG_LEN_; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    } else {  // !Found start byte but the msg contents were invalid, throw away
              // broken message
      std::vector<uint32_t> temp;
      for (int x = 1; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }

  } else {
    // !ran out of data; waiting for more
  }
  robotstatus_mutex_.unlock();
}

bool ProProtocolObject::is_connected() { return comm_base_->is_connected(); }

int ProProtocolObject::cycle_robot_mode() {
  // TODO
  return 0;
}
void ProProtocolObject::register_comm_base(const char *device) {
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
      throw(i);
    }

  } else {  // not supported device
    throw(-2);
  }
}

void ProProtocolObject::send_command(int sleeptime,
                                     std::vector<uint32_t> datalist) {
  while (true) {
    for (int x : datalist) {
      if (comm_type_ == "serial") {
        robotstatus_mutex_.lock();
        std::vector<unsigned char> write_buffer = {
            (unsigned char)startbyte_,
            (unsigned char)int(motors_speeds_[LEFT_MOTOR]),
            (unsigned char)int(motors_speeds_[RIGHT_MOTOR]),
            (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]),
            (unsigned char)requestbyte_,
            (unsigned char)x};

        write_buffer.push_back(
            (char)255 - ((unsigned char)int(motors_speeds_[LEFT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[RIGHT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]) +
                         requestbyte_ + x) %
                            255);
        comm_base_->write_to_device(write_buffer);
        robotstatus_mutex_.unlock();
      } else if (comm_type_ == "can") {
        return;  //* no CAN for rover pro
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    }
  }
}

}  // namespace RoverRobotics
