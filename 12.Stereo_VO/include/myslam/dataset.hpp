#ifndef MYSLAM_DATASET_HPP
#define MYSLAM_DATASET_HPP

#include <boost/format.hpp>

#include "myslam/common_include.hpp"
#include "myslam/camera.hpp"
#include "myslam/frame.hpp"

namespace myslam {

class Dataset {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;

  Dataset(const std::string &dataset_path) : dataset_path_(dataset_path) {
    SPDLOG_INFO("Dataset path: {}", dataset_path_);
    cameras_.reserve(4);
  }

  bool init(){
    // read camera intrinsics and extrinsics
    std::ifstream fin(dataset_path_ + "/calib.txt");
    if (!fin) {
      SPDLOG_ERROR("cannot find {}/calib.txt!", dataset_path_);
      return false;
    }

    for (int i = 0; i < 4; ++i) {
      char camera_name[3];
      for (int k = 0; k < 3; ++k) {
        fin >> camera_name[k];
      }
      double projection_data[12];
      for (int k = 0; k < 12; ++k) {
          fin >> projection_data[k];
      }
      Eigen::Mat33 K;
      K << projection_data[0], projection_data[1], projection_data[2],
           projection_data[4], projection_data[5], projection_data[6],
           projection_data[8], projection_data[9], projection_data[10];
      Eigen::Vec3 t;
      t << projection_data[3], projection_data[7], projection_data[11];
      t= K.inverse() * t;
      K = K * 0.5;
      Camera::Ptr camera = std::make_shared<Camera>(
        K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), Sophus::SE3d(Sophus::SO3d(), t));
      cameras_.push_back(camera);
      SPDLOG_INFO("Camera {} extrinsics: {}", i, t.transpose());
    }

    fin.close();
    current_image_index_ = 0;
    return true;
  }

  Frame::Ptr NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    // Read images
    SPDLOG_INFO("Read! {}", (fmt % dataset_path_ % 0 % current_image_index_).str());
    img_left_ = cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    img_right_ = cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    if (img_left_.data == nullptr || img_right_.data == nullptr) {
      SPDLOG_WARN("cannot find images at index {}", current_image_index_);
      return nullptr;
    }

    // Resize images
    cv::resize(img_left_, img_left_resized_, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    cv::resize(img_right_, img_right_resized_, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    cv::imshow("left", img_left_resized_);
    cv::imshow("right", img_right_resized_);
    cv::waitKey(1);

    current_image_index_++;

    Frame::Ptr new_frame = Frame::createFrame();
    return new_frame;
  }

  int getCurrentImageIndex() const { return current_image_index_; }

private:
  const std::string dataset_path_;
  int current_image_index_ = 0;

  std::vector<Camera::Ptr> cameras_;
  cv::Mat img_left_, img_right_;
  cv::Mat img_left_resized_, img_right_resized_;
};
} // namespace myslam
#endif // MYSLAM_DATASET_HPP
