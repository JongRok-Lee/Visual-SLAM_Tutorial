#ifndef MYSLAM_VISUAL_ODOMETRY_HPP
#define MYSLAM_VISUAL_ODOMETRY_HPP

#include "myslam/common_include.hpp"
#include "myslam/dataset.hpp"

namespace myslam {
class VisualOdometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;

  VisualOdometry(const std::string &config_path) : config_file_path_(config_path) {}

  bool init() {
    // read from config file
    std::cout << "VisualOdometry Init!\n";
    config_ = YAML::LoadFile(config_file_path_);

    dataset_ = std::make_shared<Dataset>(config_["dataset_dir"].as<std::string>());
    if (!dataset_->init()) {
      return false;
    }


    return true;
  }

  void run() {
    while (step()) {
      SPDLOG_INFO("Frame: {}", dataset_->getCurrentImageIndex());
    }
  }
  bool step(){
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    return true;
  }

private:
  bool inited_ = false;
  const std::string config_file_path_;

  YAML::Node config_;
  Dataset::Ptr dataset_ = nullptr;
};
} // namespace myslam

#endif // MYSLAM_VISUAL_ODOMETRY_HPP
