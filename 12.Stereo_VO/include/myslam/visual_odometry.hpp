#ifndef MYSLAM_VISUAL_ODOMETRY_HPP
#define MYSLAM_VISUAL_ODOMETRY_HPP

#include "myslam/backend.hpp"
#include "myslam/common_include.hpp"
#include "myslam/dataset.hpp"
#include "myslam/frontend.hpp"
#include "myslam/viewer.hpp"

namespace myslam {
class VisualOdometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;

  VisualOdometry(std::string &config_path);

  bool Init();

  void Run();

  bool Step();

  FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
  bool inited_ = false;
  std::string config_file_path_;

  Frontend::Ptr frontend_ = nullptr;
  Backend::Ptr backend_ = nullptr;
  Map::Ptr map_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;

  // dataset
  Dataset::Ptr dataset_ = nullptr;
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_HPP
