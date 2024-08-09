#include <iostream>
#include <gflags/gflags.h>

#include "myslam/visual_odometry.hpp"



int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_file = "/home/jr/Visual-SLAM_Tutorial/12.Stereo_VO/config/default.yaml";

  myslam::VisualOdometry::Ptr vo = std::make_shared<myslam::VisualOdometry>(config_file);

  spdlog::set_level(spdlog::level::err);
  vo->Init();
  vo->Run();

  return 0;
}