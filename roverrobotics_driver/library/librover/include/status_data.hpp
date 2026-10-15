#include <chrono>
#pragma once
namespace RoverRobotics {

/* Why the control loop will not move the robot. SOFT_ESTOP is the software
   e-stop only; the hardware e-stops are invisible to this driver. */
enum MotionBlocked {
  MOTION_OK = 0,
  MOTION_SOFT_ESTOP = 1,
  MOTION_VESC_STALE = 2,
  MOTION_CMD_TIMEOUT = 3,
};
struct robotData {
  // Motor Infos
  signed short int motor1_id;
  float motor1_rpm;
  float motor1_current;
  signed short int motor1_temp;
  signed short int motor1_mos_temp;
  signed short int motor2_id;
  float motor2_rpm;
  float motor2_current;
  signed short int motor2_temp;
  signed short int motor2_mos_temp;
  signed short int motor3_id;
  float motor3_rpm;
  float motor3_current;
  signed short int motor3_temp;
  signed short int motor3_mos_temp;
  signed short int motor4_id;
  float motor4_rpm;
  float motor4_current;
  signed short int motor4_temp;
  signed short int motor4_mos_temp;

  // Battery Infos
  float battery1_voltage;
  float battery2_voltage;
  signed short int battery1_temp;
  signed short int battery2_temp;
  float battery1_current;
  float battery2_current;
  float battery1_SOC;
  float battery2_SOC;
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

  // CAN feedback age per VESC, indexed by VESC_IDS 1..4; [0] unused. Capped at
  // 100000 ms, which the driver reports as "none received". 0 on a serial robot.
  float vesc_age_ms[5] = {0, 0, 0, 0, 0};
  // 0 = every VESC fresh, else the id of the stalest one. The robot holds all
  // motors stopped while this is non-zero, so it must be visible outside.
  unsigned short int stale_vesc = 0;
  // why the control loop is refusing to move, see MotionBlocked. SOFT e-stop
  // only: the driver cannot see the hardware e-stops at all.
  unsigned short int motion_blocked = 0;
  // age of the last velocity command when the loop last looked, ms
  float cmd_age_ms = 0;
};
}  // namespace RoverRobotics
