#ifndef DIRECT_METHOD_HPP
#define DIRECT_METHOD_HPP
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

#include<experimental/filesystem>

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class JacobianAccumulator {
public:
  JacobianAccumulator(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const std::vector<double> depth_ref,
    Sophus::SE3d &T21);

  /// accumulate jacobians in a range
  void accumulate_jacobian(const cv::Range &range);

  /// get hessian matrix
  Matrix6d hessian() const;

  /// get bias
  Vector6d bias() const;

  /// get total cost
  double cost_func() const;

  /// get projected points
  VecVector2d projected_points() const;

  /// reset h, b, cost to zero
  void reset();

private:
  const cv::Mat &img1_;
  const cv::Mat &img2_;
  const VecVector2d &px_ref_;
  const std::vector<double> depth_ref_;
  Sophus::SE3d &T21_;
  VecVector2d projection_;

  std::mutex hessian_mutex_;
  Matrix6d H_ = Matrix6d::Zero();
  Vector6d b_ = Vector6d::Zero();
  double cost_ = 0.0;
};

float GetPixelValue(const cv::Mat &img, float x, float y);

void DirectPoseEstimationSingleLayer(
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  const std::vector<double> depth_ref,
  Sophus::SE3d &T21);

void DirectPoseEstimationMultiLayer(
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  const std::vector<double> depth_ref,
  Sophus::SE3d &T21);


#endif // DIRECT_METHOD_HPP