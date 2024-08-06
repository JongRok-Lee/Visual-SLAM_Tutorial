#include <iostream>
#include <gflags/gflags.h>

#include "myslam/visual_odometry.hpp"


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_file = "./config/default.yaml";

  myslam::VisualOdometry::Ptr vo = std::make_shared<myslam::VisualOdometry>(config_file);
  vo->init();
  vo->run();

  return 0;
}