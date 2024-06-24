#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

class OpticalFlowTracker {
public:
  OpticalFlowTracker(const cv::Mat &img1, const cv::Mat &img2,
                     const std::vector<cv::KeyPoint> &kp1,
                     std::vector<cv::KeyPoint> &kp2,
                     std::vector<bool> &success,
                     bool inverse = true,
                     bool has_initial = false);

  void calculateOpticalFlow(const cv::Range &range);

private:
  const cv::Mat &img1_;
  const cv::Mat &img2_;
  const std::vector<cv::KeyPoint> &kp1_;
  std::vector<cv::KeyPoint> &kp2_;
  std::vector<bool> &success_;
  bool inverse_;
  bool has_initial_;
};

float GetPixelValue(const cv::Mat &img, float x, float y);

void OpticalFlowSingleLevel(const cv::Mat &img1, const cv::Mat &img2,
                            const std::vector<cv::KeyPoint> &kp1,
                            std::vector<cv::KeyPoint> &kp2,
                            std::vector<bool> &success,
                            bool inverse = false, bool has_initial = false);
void OpticalFlowMultiLevel(const cv::Mat &img1, const cv::Mat &img2,
                           const std::vector<cv::KeyPoint> &kp1,
                           std::vector<cv::KeyPoint> &kp2,
                           std::vector<bool> &success,
                           bool inverse = false);
#endif // OPTICAL_FLOW_HPP