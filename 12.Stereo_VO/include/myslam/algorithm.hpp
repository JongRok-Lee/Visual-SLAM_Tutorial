#ifndef MYSLAM_ALGORITHM_HPP
#define MYSLAM_ALGORITHM_HPP

#include "myslam/common_include.hpp"

namespace myslam {

bool triangulation(const std::vector<Sophus::SE3d> &poses,
                   const std::vector<Eigen::Vec3> points,
                   Eigen::Vec3 &pt_world)
{
  Eigen::MatXX A(2 * poses.size(), 4);
  Eigen::VecX b(2 * poses.size());
  b.setZero();

  for (size_t i = 0; i < poses.size(); ++i) {
    Eigen::Mat34 m = poses[i].matrix3x4();
    A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
    A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
  }
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

  if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
    return true;
  }

  return false;
}

inline Eigen::Vec2 toVec2(const cv::Point2f p) {
  return Eigen::Vec2(p.x, p.y);
  }
}

#endif // MYSLAM_ALGORITHM_HPP
