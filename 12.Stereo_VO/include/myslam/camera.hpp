#ifndef MYSLAM_CAMERA_HPP
#define MYSLAM_CAMERA_HPP

#include "myslam/common_include.hpp"

namespace myslam {

class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Camera> Ptr;

  Camera() = default;
  Camera(double fx, double fy, double cx, double cy, double base_line, const Sophus::SE3d &pose)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), base_line_(base_line), pose_(pose) {
    pose_inv_ = pose_.inverse();
  }

  Sophus::SE3d pose() const { return pose_; }

  // Return Intrinsics matrix
  Eigen::Mat33 K() const {
    Eigen::Mat33 K;
    K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return K;
  }

  // coordinate transform: world, camera, pixel
  Eigen::Vec3 world2camera(const Eigen::Vec3 &p_w, const Sophus::SE3d &T_c_w) {
    return pose_ * T_c_w * p_w;
  }
  Eigen::Vec3 camera2world(const Eigen::Vec3 &p_c, const Sophus::SE3d &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
  }
  Eigen::Vec2 camera2pixel(const Eigen::Vec3 &p_c) {
    return Eigen::Vec2(
      fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
      fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
  }
  Eigen::Vec3 pixel2camera(const Eigen::Vec2 &p_p, double depth = 1) {
    return Eigen::Vec3(
      (p_p(0, 0) - cx_) * depth / fx_,
      (p_p(1, 0) - cy_) * depth / fy_,
      depth
    );
  }
  Eigen::Vec3 pixel2world(const Eigen::Vec2 &p_p, const Sophus::SE3d &T_c_w, double depth = 1) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
  }
  Eigen::Vec2 world2pixel(const Eigen::Vec3 &p_w, const Sophus::SE3d &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
  }

public:
  // Camera intrinsics
  double fx_ = 0;
  double fy_ = 0;
  double cx_ = 0;
  double cy_ = 0;
  double base_line_ = 0;
  // Extrinsics
  Sophus::SE3d pose_;
  Sophus::SE3d pose_inv_;
};
} // namespace myslam
#endif // MYSLAM_CAMERA_HPP