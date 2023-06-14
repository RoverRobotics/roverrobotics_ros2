#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <mutex>

namespace Utilities {
/* classes */
class PersistentParams;
}  // namespace Utilities

class Utilities::PersistentParams {
 private:
  std::string robot_param_path_;
  std::fstream file_rw_;

  std::vector<std::pair<std::string, double>> read_params_from_file_();

  std::vector<std::string> split_(std::string str, std::string token);
  std::mutex file_mutex;

 public:
  PersistentParams(std::string robot_param_path);
  void write_param(std::string key, double value);
  std::optional<double> read_param(std::string key);

};