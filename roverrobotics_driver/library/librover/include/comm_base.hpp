#pragma once

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "status_data.hpp"
#include "utils.hpp"
#include "global_error_constants.hpp"


namespace RoverRobotics {
class CommBase;
}
class RoverRobotics::CommBase {
 public:
  /*
   * @brief Pure Virtual Interface of Write To Communication Device.
   * The implementation of this function should accept a vector of unsigned
   * int32 and convert it to a suitable format for the connected device before
   * sending it out
   * @param vector<uint32> to convert and write to device
   */
  virtual void write_to_device(std::vector<uint8_t>) = 0;
  /*
   * @brief Pure Virtual Interface of Read From Communication Device.
   * The implementation of this function should read from the connected
   * communication device buffer and convert it to a vector of unsigned int32.
   * There are no decoding of message in this function as it should only be
   * inside the callback function accepted from this method.
   * @param callbackfunction to decode the message
   */
  virtual void read_device_loop(std::function<void(std::vector<uint8_t>)>) = 0;
  /*
   * @brief Pure Virtual Interface to check if the communication device is still
   * connected. The implementation of this function should check the status of
   * the communication device file descriptor and report it state
   * @return bool file descriptor state
   */
  virtual bool is_connected() = 0;
};