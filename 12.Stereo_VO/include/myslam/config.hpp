#ifndef MYSLAM_CONFIG_HPP
#define MYSLAM_CONFIG_HPP

#include "myslam/common_include.hpp"
#include <yaml-cpp/yaml.h>

namespace myslam {

class Config {
private:
  // private constructor makes a singleton
  Config() {}
public:
  ~Config();  // close the file when deconstructing

  // set a new config file
  static bool SetParameterFile(const std::string &filename);

  // access the parameter values
  template <typename T>
  static T Get(const std::string &key) {
    return T(Config::config_->file_[key].as<T>());
  }

private:
  static std::shared_ptr<Config> config_;
  YAML::Node file_;
};
}  // namespace myslam

#endif  // MYSLAM_CONFIG_H
