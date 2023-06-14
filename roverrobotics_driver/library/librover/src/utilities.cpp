#include "utilities.hpp"
#include <algorithm>
namespace Utilities {

PersistentParams::PersistentParams(std::string robot_param_path) {
  robot_param_path_ = robot_param_path;
}

std::vector<std::pair<std::string, double>>
PersistentParams::read_params_from_file_() {
  /* create the return vector */
  std::vector<std::pair<std::string, double>> return_data;
  
  /* open file */
  file_mutex.lock();
  file_rw_.open(robot_param_path_);

  /* read in all the lines, 1 key/pair per line, ":" delimited */
  if (file_rw_.is_open()) {
    std::string line;
    while (std::getline(file_rw_, line)) {
      
      /* split the line into 1 key and 1 value*/
      std::vector<std::string> key_and_value = split_(line, ":");

      /* extract key and value */
      std::string key = key_and_value.front();
      double value = std::stod(key_and_value.back());

      return_data.push_back(std::pair<std::string, double>(key, value));
    }
  }else{
    std::cout << "Warning: ~/robot.config file not found, persistent trim disabled" << std::endl;
  }

  file_rw_.close();
  file_mutex.unlock();
  return return_data;
}

void PersistentParams::write_param(std::string param_name, double value){
  /* determine if param exists already */
  auto param_pairs = read_params_from_file_();

  /* build an array of just the keys */
  std::vector<std::string> keys_only;
  for(auto it = begin(param_pairs); it != end(param_pairs); ++it) keys_only.push_back(it->first);

  /* search the keys for match to the param_name */
  auto location = std::find(keys_only.begin(), keys_only.end(), param_name);

  /* parameter already exists, delete from the array */
  if(location != keys_only.end()){
    param_pairs.erase(param_pairs.begin() + (location - keys_only.begin()));
  }

  /* update the pairs array then rewrite the file */
  param_pairs.push_back(std::pair<std::string, double>(param_name, value));

  /* clear the output file, then write it */
  file_mutex.lock();
  file_rw_.open(robot_param_path_, std::ifstream::out | std::ifstream::trunc);

  if(!file_rw_.is_open()){
    std::cout << "Failed to open persistent param file" << std::endl;
    return;
  }

  for(auto pair: param_pairs){
    file_rw_ << pair.first << ":" << pair.second << std::endl;
  }

  file_rw_.close();
  file_mutex.unlock();
  return;
}

std::optional<double> PersistentParams::read_param(std::string param_name){
  /* determine if param exists already */
  auto param_pairs = read_params_from_file_();

  /* build an array of just the keys */
  std::vector<std::string> keys_only;
  for(auto it = begin(param_pairs); it != end(param_pairs); ++it) keys_only.push_back(it->first);

  /* search the keys for match to the param_name */
  auto location = std::find(keys_only.begin(), keys_only.end(), param_name);

  /* parameter already exists, delete from the array */
  if(location == keys_only.end()){
    return {};
  }

  return param_pairs[location - keys_only.begin()].second;

}

std::vector<std::string> PersistentParams::split_(std::string str, std::string token) {
  std::vector<std::string> result;
  while (str.size()) {
    int index = str.find(token);
    if (index != std::string::npos) {
      result.push_back(str.substr(0, index));
      str = str.substr(index + token.size());
      if (str.size() == 0) result.push_back(str);
    } else {
      result.push_back(str);
      str = "";
    }
  }
  return result;
}

}  // namespace Utilities
