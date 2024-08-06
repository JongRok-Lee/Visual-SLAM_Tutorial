#ifndef MYSLAM_FRAME_HPP
#define MYSLAM_FRAME_HPP

#include "myslam/common_include.hpp"
#include "myslam/camera.hpp"

namespace myslam {

struct Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frame> Ptr;

  Frame() = default;
  ~Frame() = default;

  static std::shared_ptr<Frame> createFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame = std::make_shared<Frame>();
    new_frame->id_ = factory_id++;
    return new_frame;
  }

public:
  unsigned long id_ = 0;           // id of this frame
  unsigned long keyframe_id_ = 0;  // id of key frame

};
} // namespace myslam

#endif // MYSLAM_FRAME_HPP