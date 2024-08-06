#ifndef MYSLAM_FRONTEND_HPP
#define MYSLAM_FRONTEND_HPP

#include "myslam/common_include.hpp"
#include "myslam/frame.hpp"
#include "myslam/feature.hpp"

namespace myslam {

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frontend> Ptr;

  Frontend(const YAML::Node &config) : config_(config) {
    gftt_ = cv::GFTTDetector::create(config_["num_features"].as<int>(), 0.01, 20);
    num_features_init_ = config_["num_features_init"].as<int>();
    num_features_ = config_["num_features"].as<int>();
  }

  bool addFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;
    int num_features_left = detectFeatures();
    int num_coor_features = findFeaturesInRight();
    if (num_coor_features < num_features_init_) {
      return false;
    }

    last_frame_ = current_frame_;
    return true;
  }

  int detectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
      cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                    feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
      current_frame_->features_left_.push_back(
        std::make_shared<Feature>(current_frame_, kp));
      cnt_detected++;
    }

    cv::Mat outImg;
    cv::drawKeypoints(current_frame_->left_img_, keypoints, outImg);
    cv::imshow("left feature", outImg);
    cv::waitKey(1);

    SPDLOG_INFO("Detect {} new features", cnt_detected);
    return cnt_detected;
  }

  int findFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
      kps_left.push_back(kp->position_.pt);
      auto mp = kp->map_point_.lock();
      if (mp) {
        // use projected points as initial guess
        auto px = camera_right_->world2pixel(mp->pos_, current_frame_->getPose());
        kps_right.push_back(cv::Point2f(px[0], px[1]));
      } else {
        // use same pixel in left iamge
        kps_right.push_back(kp->position_.pt);
      }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
      current_frame_->left_img_, current_frame_->right_img_, kps_left,
      kps_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                      0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i]) {
        cv::KeyPoint kp(kps_right[i], 7);
        Feature::Ptr feat = std::make_shared<Feature>(current_frame_, kp);
        feat->is_on_left_image_ = false;
        current_frame_->features_right_.push_back(feat);
        num_good_pts++;
      } else {
        current_frame_->features_right_.push_back(nullptr);
      }
    }

    // visualization
    std::vector<cv::KeyPoint> keypoints;
    for (auto &feat : current_frame_->features_right_) {
      if (feat) {
        keypoints.push_back(feat->position_);
      }
    }
    cv::Mat outImg;
    cv::drawKeypoints(current_frame_->right_img_, keypoints, outImg);
    cv::imshow("right feature", outImg);
    cv::waitKey(0);

    SPDLOG_INFO("Find {} in the right image", num_good_pts);
    return num_good_pts;
  }


private:
  // data
  FrontendStatus status_ = FrontendStatus::INITING;
  const YAML::Node config_;

  Frame::Ptr current_frame_ = nullptr;
  Frame::Ptr last_frame_ = nullptr;
  Camera::Ptr camera_left_ = nullptr;
  Camera::Ptr camera_right_ = nullptr;

  Sophus::SE3d relative_motion_;

  int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

  // params
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  // utilities
  cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};


} // namespace myslam
#endif // MYSLAM_FRONTEND_HPP
