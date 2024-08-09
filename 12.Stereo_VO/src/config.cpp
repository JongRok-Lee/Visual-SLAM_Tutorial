#include "myslam/config.hpp"

namespace myslam {
bool Config::SetParameterFile(const std::string &filename) {
  if (config_ == nullptr) {
    config_ = std::shared_ptr<Config>(new Config);
  }
  config_->file_ = YAML::LoadFile(filename);
  return true;
}

Config::~Config() = default;

std::shared_ptr<Config> Config::config_ = nullptr;

} // namespace myslam
