#ifndef MYSLAM_FRAME_HPP
#define MYSLAM_FRAME_HPP

#include "myslam/common_include.hpp"
#include "myslam/camera.hpp"
#include "myslam/feature.hpp"

namespace myslam {
struct MapPoint;
// struct Feature;

struct Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frame> Ptr;

  Frame() = default;
  Frame(long id, double time_stamp, const Sophus::SE3d &pose, const cv::Mat &left, const cv::Mat &right)
      : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}
  ~Frame() = default;

  static std::shared_ptr<Frame> createFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame = std::make_shared<Frame>();
    new_frame->id_ = factory_id++;
    return new_frame;
  }

  void setKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
  }

  // set and get pose, thread safe
  Sophus::SE3d getPose() {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    return pose_;
  }

  void setPose(const Sophus::SE3d &pose) {
      std::unique_lock<std::mutex> lck(pose_mutex_);
      pose_ = pose;
  }

public:
  unsigned long id_ = 0;           // id of this frame
  unsigned long keyframe_id_ = 0;  // id of key frame
  bool is_keyframe_ = false;       // whether a keyframe
  double time_stamp_;              // 时间戳，暂不使用
  Sophus::SE3d pose_;              //
  std::mutex pose_mutex_;          //
  cv::Mat left_img_, right_img_;   // stereo images

  // extracted features in left image
  std::vector<std::shared_ptr<Feature>> features_left_;
  // corresponding features in right image, set to nullptr if no corresponding
  std::vector<std::shared_ptr<Feature>> features_right_;

};
} // namespace myslam

#endif // MYSLAM_FRAME_HPP