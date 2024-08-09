#ifndef MYSLAM_FRAME_HPP
#define MYSLAM_FRAME_HPP

#include "myslam/camera.hpp"
#include "myslam/common_include.hpp"

namespace myslam {

// forward declare
struct MapPoint;
struct Feature;

struct Frame {
public:
  Frame() {}

  Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

  SE3 Pose() {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    return pose_;
  }

  void SetPose(const SE3 &pose) {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    pose_ = pose;
  }

  void SetKeyFrame();

  static std::shared_ptr<Frame> CreateFrame();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frame> Ptr;

  unsigned long id_ = 0;           // id of this frame
  unsigned long keyframe_id_ = 0;  // id of key frame
  bool is_keyframe_ = false;       // 是否为关键帧
  double time_stamp_;              // 时间戳，暂不使用
  SE3 pose_;                       // Tcw 形式Pose
  std::mutex pose_mutex_;          // Pose数据锁
  cv::Mat left_img_, right_img_;   // stereo images

  // extracted features in left image
  std::vector<std::shared_ptr<Feature>> features_left_;
  std::vector<std::shared_ptr<Feature>> features_right_;
};

}  // namespace myslam

#endif  // MYSLAM_FRAME_HPP
