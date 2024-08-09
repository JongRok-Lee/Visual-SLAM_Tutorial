#ifndef MYSLAM_DATASET_HPP
#define MYSLAM_DATASET_HPP

#include "myslam/camera.hpp"
#include "myslam/common_include.hpp"
#include "myslam/frame.hpp"

namespace myslam {

class Dataset {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;
  Dataset(const std::string& dataset_path);

  bool Init();

  Frame::Ptr NextFrame();

  Camera::Ptr GetCamera(int camera_id) const {
      return cameras_.at(camera_id);
  }

private:
  std::string dataset_path_;
  int current_image_index_ = 0;

  std::vector<Camera::Ptr> cameras_;
};
}  // namespace myslam

#endif // MYSLAM_DATASET_HPP
