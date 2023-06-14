#include <stdio.h>

#include <memory>

#include "protocol_mini.hpp"
#include "protocol_pro_2.hpp"
#include "protocol_pro.hpp"
#include "protocol_zero_2.hpp"
#include "time.h"
using namespace RoverRobotics;

void print_status(RoverRobotics::robotData &robotdata) {
  std::cerr << "Robot Data " << std::endl
            << "motor 1 id " << robotdata.motor1_id << std::endl
            << "motor1_rpm " << robotdata.motor1_rpm << std::endl
            << "motor1_current " << robotdata.motor1_current << std::endl
            << "motor1_temp " << robotdata.motor1_temp << std::endl
            << "motor1_mos_temp " << robotdata.motor1_mos_temp << std::endl
            << "motor2_id " << robotdata.motor2_id << std::endl
            << "motor2_rpm " << robotdata.motor2_rpm << std::endl
            << "motor2_current " << robotdata.motor2_current << std::endl
            << "motor2_temp " << robotdata.motor2_temp << std::endl
            << "motor2_mos_temp " << robotdata.motor2_mos_temp << std::endl
            << "motor3_id " << robotdata.motor3_id << std::endl
            << "motor3_rpm " << robotdata.motor3_rpm << std::endl
            << "motor3_current " << robotdata.motor3_current << std::endl
            << "motor3_temp " << robotdata.motor3_temp << std::endl
            << "motor3_mos_temp " << robotdata.motor3_mos_temp << std::endl
            << "motor4_id " << robotdata.motor4_id << std::endl
            << "motor4_rpm " << robotdata.motor4_rpm << std::endl
            << "motor4_current " << robotdata.motor4_current << std::endl
            << "motor4_temp " << robotdata.motor4_temp << std::endl
            << "motor4_mos_temp " << robotdata.motor4_mos_temp << std::endl
            << "battery1_voltage " << robotdata.battery1_voltage << std::endl
            << "battery2_voltage " << robotdata.battery2_voltage << std::endl
            << "battery1_temp " << robotdata.battery1_temp << std::endl
            << "battery2_temp " << robotdata.battery2_temp << std::endl
            << "battery1_current " << robotdata.battery1_current << std::endl
            << "battery2_current " << robotdata.battery2_current << std::endl
            << "battery1_SOC " << robotdata.battery1_SOC << std::endl
            << "battery2_SOC " << robotdata.battery2_SOC << std::endl
            << "battery1_fault_flag " << robotdata.battery1_fault_flag
            << std::endl
            << "battery2_fault_flag " << robotdata.battery2_fault_flag
            << std::endl
            << "robot_guid " << robotdata.robot_guid << std::endl
            << "robot_firmware " << robotdata.robot_firmware << std::endl
            << "robot_fault_flag " << robotdata.robot_fault_flag << std::endl
            << "robot_fan_speed " << robotdata.robot_fan_speed << std::endl
            << "robot_speed_limit " << robotdata.robot_speed_limit << std::endl
            << "motor3_angle " << robotdata.motor3_angle << std::endl
            << "motor3_sensor1 " << robotdata.motor3_sensor1 << std::endl
            << "motor3_sensor2 " << robotdata.motor3_sensor2 << std::endl
            << "linear_vel " << robotdata.linear_vel << std::endl
            << "angular_vel " << robotdata.angular_vel << std::endl
            << "cmd_linear_vel " << robotdata.cmd_linear_vel << std::endl
            << "cmd_angular_vel " << robotdata.cmd_angular_vel << std::endl
	    << "Firmware" << robotdata.robot_firmware << std::endl;
}

int main() {
  // Control::pid_gains testgains_ = {0.0009, 0, 0.00007};
  Control::pid_gains testgains_ = {0, 0, 0};

  Control::robot_motion_mode_t robot_mode = Control::INDEPENDENT_WHEEL;
  Control::angular_scaling_params angular_scaling_params_ = {0, 1, 0, 1, 1};
  // std::unique_ptr<BaseProtocolObject> robot_ =
  //     std::make_unique<MiniProtocolObject>(
  //         "can0", "can", robot_mode, testgains_,
  //         angular_scaling_params_);
  // std::make_unique<Pro2ProtocolObject>(
  //       "can0", "can", robot_mode, testgains_,
  //       angular_scaling_params_);
  std::unique_ptr<BaseProtocolObject> robot_ =
      std::make_unique<ProProtocolObject>("/dev/rover-pro", "serial",
                                            robot_mode, testgains_);
  //robot_->cycle_robot_mode();

  while (true) {
    auto status = robot_->status_request();
    // std::cout << status.angular_vel << std::endl;

    // auto connected = robot_->is_connected();
    // std::cout << "connected:  " << connected << std::endl;

    auto info = robot_->info_request();
    print_status(info);
    //double controlarray[2] = {1, 0};
    //robot_->set_robot_velocity(controlarray);
    // robot_->cycle_robot_mode();

    // robot_->send_estop(true);

    // robot_->update_drivetrim(0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
