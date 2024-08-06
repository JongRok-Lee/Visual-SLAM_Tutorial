#ifndef MYSLAM_VISUAL_ODOMETRY_HPP
#define MYSLAM_VISUAL_ODOMETRY_HPP

#include "myslam/common_include.hpp"

namespace myslam {
class VisualOdometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;

  VisualOdometry(const std::string &config_path) : config_file_path_(config_path) {}
  bool init(){ return true; }
  void run(){};
  bool step(){ return true; }

private:
  bool inited_ = false;
  std::string config_file_path_;
};
} // namespace myslam

#endif // MYSLAM_VISUAL_ODOMETRY_HPP
