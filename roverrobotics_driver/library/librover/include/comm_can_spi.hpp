#pragma once
#include "comm_base.hpp"

extern "C"{
#include <mpsse.h>
}

#define FTDI_CREATION_FAIL -1
#define OPEN_DEVICE_FAIL -1
#define CLOSE_DEVICE_FAIL -1

// MCP25625 Commands and Registers
#define MCP_CMD_WRITE 0x02
#define MCP_CMD_READ 0x03
#define MCP_REG_CANCNTRL 0x0F
#define MCP_REG_CANSTAT 0x0E
#define MCP_REG_REC 0x1D
#define MCP_REG_TEC 0x1C
#define MCP_REG_EFLG 0x2D
#define MCP_REG_TXB0 0x30

namespace RoverRobotics {
class CommCanSPI;
}
class RoverRobotics::CommCanSPI : public RoverRobotics::CommBase {
 public:
  /*
   * @brief Constructor For Can Communication
   * This contructor accepts a Can device path, a callback function for
   * reading from the Can device and a std::vector<uint32_t> for settings.
   * Callback funcion is automatically started as a thread inside this
   * contructor
   *
   * @param device the device path
   * @param callbackfunction
   * @param settings
   */
  CommCanSPI(const char *device, std::function<void(std::vector<uint8_t>)>,
          std::vector<uint8_t>);
  /*
   * @brief Write data to Can Device
   * by accepting a vector of unsigned int 32 and convert it to a byte stream
   * @param msg message to convert and write to device
   */
  void write_to_device(std::vector<uint8_t> msg);
  /*
   * @brief Read data from Can Device
   * by reading the current device buffer then convert to a vector of unsigned
   * int 32.
   * @param callback to process the unsigned int 32.
   */
  void read_device_loop(std::function<void(std::vector<uint8_t>)>);
  /*
   * @brief Check if Can device is still connected by check the state of the
   * file descriptor
   * @return bool file descriptor state
   */
  bool is_connected();
  struct mpsse_context *ftdi = NULL;

 private:
  struct sockaddr_can addr;  // CAN Address
  struct can_frame frame;
  struct can_frame robot_frame;
  struct ifreq ifr;
  int fd;
  int read_size_;
  int Can_port_;
  const int CAN_MSG_SIZE_ = 9;
  std::atomic<bool> is_connected_;
  std::mutex Can_write_mutex_;
  std::thread Can_read_thread_;
  const int TIMEOUT_MS_ = 1000;  // 1 sec timeout
};
