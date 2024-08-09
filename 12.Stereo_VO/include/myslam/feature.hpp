#ifndef MYSLAM_FEATURE_HPP
#define MYSLAM_FEATURE_HPP

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.hpp"

namespace myslam {
struct Frame;
struct MapPoint;

struct Feature {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Feature> Ptr;
  Feature() = default;
  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
    : frame_(frame), position_(kp) {}

public:
  std::weak_ptr<Frame> frame_;
  cv::KeyPoint position_;
  std::weak_ptr<MapPoint> map_point_;

  bool is_outlier_ = false;
  bool is_on_left_image_ = true;
};
}  // namespace myslam

#endif  // MYSLAM_FEATURE_HPP
