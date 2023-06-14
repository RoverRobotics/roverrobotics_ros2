
#include "comm_serial.hpp"

namespace RoverRobotics {
CommSerial::CommSerial(const char *device,
                       std::function<void(std::vector<uint8_t>)> parsefunction,
                       std::vector<uint8_t> setting) {
  // open serial port at specified port
  serial_port_ = open(device, 02);

  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0) {
    std::cerr << "error";
    throw(-1);
    return;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size tty.c_cflag                           
  tty.c_cflag |= CS8;      // |= CS8; // 8 bits per byte (most common)
                           
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 0;  // remove wait time

  // Set in/out baud rate
  int baud =
      (setting[0] << 24) + (setting[1] << 16) + (setting[2] << 8) + setting[3];
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  read_size_ = (int)setting[4];
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
    std::cerr << "error saving tty settings";
    throw(-1);
    return;
  }
  is_connected_ = false;
  serial_read_thread_ = std::thread(
      [this, parsefunction]() { this->read_device_loop(parsefunction); });
}

void CommSerial::write_to_device(std::vector<uint8_t> msg) {
  serial_write_mutex_.lock();
  if (serial_port_ >= 0) {
    uint8_t write_buffer[msg.size()];
    for (int x = 0; x < msg.size(); x++) {
      write_buffer[x] = msg[x];
    }
    write(serial_port_, write_buffer, msg.size());
  }
  serial_write_mutex_.unlock();
}

void CommSerial::read_device_loop(
    std::function<void(std::vector<uint8_t>)> parsefunction) {
  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  while (true) {
    uint8_t read_buf[read_size_];
    int num_bytes = read(serial_port_, &read_buf, read_size_);
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    if (num_bytes <= 0) {
      if ((time_now - time_last).count() > TIMEOUT_MS_) {
        is_connected_ = false;
      }
      continue;
    }
    is_connected_ = true;
    time_last = time_now;
    static std::vector<uint8_t> output;
    for (int x = 0; x < num_bytes; x++) {
      output.push_back(read_buf[x]);
    }
    parsefunction(output);
    output.clear();
  }
}

bool CommSerial::is_connected() { return (is_connected_); }

}  // namespace RoverRobotics
