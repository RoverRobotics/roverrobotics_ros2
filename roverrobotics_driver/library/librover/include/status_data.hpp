#include <chrono>
#pragma once
namespace RoverRobotics {
struct robotData {
  // Motor Infos
  signed short int motor1_id;
  float motor1_rpm;
  signed short int motor1_current;
  signed short int motor1_temp;
  signed short int motor1_mos_temp;
  signed short int motor2_id;
  float motor2_rpm;
  signed short int motor2_current;
  signed short int motor2_temp;
  signed short int motor2_mos_temp;
  signed short int motor3_id;
  float motor3_rpm;
  signed short int motor3_current;
  signed short int motor3_temp;
  signed short int motor3_mos_temp;
  signed short int motor4_id;
  float motor4_rpm;
  signed short int motor4_current;
  signed short int motor4_temp;
  signed short int motor4_mos_temp;

  // Battery Infos
  unsigned short int battery1_voltage;
  unsigned short int battery2_voltage;
  signed short int battery1_temp;
  signed short int battery2_temp;
  unsigned short int battery1_current;
  unsigned short int battery2_current;
  unsigned short int battery1_SOC;
  unsigned short int battery2_SOC;
  unsigned short int battery1_fault_flag;
  unsigned short int battery2_fault_flag;

  // Robot FEEDBACK Infos
  unsigned short int robot_guid;
  unsigned short int robot_firmware;
  unsigned short int robot_fault_flag;
  unsigned short int robot_fan_speed;
  unsigned short int robot_speed_limit;

  // Flipper Infos
  unsigned short int motor3_angle;
  unsigned short int motor3_sensor1;
  unsigned short int motor3_sensor2;

  // Robot Info
  double linear_vel;
  double angular_vel;

  // Velocity Info
  double cmd_linear_vel;
  double cmd_angular_vel;
  std::chrono::milliseconds cmd_ts;
};
}  // namespace RoverRobotics