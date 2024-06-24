#include "optical_flow.hpp"

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cout << "Usage: optical_flow path_to_img1 path_to_img2" << std::endl;
    return 1;
  }

  cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

  if (img1.data == nullptr || img2.data == nullptr) {
    std::cout << "Cannot open images!" << std::endl;
    return 1;
  }

  // Detect features (GFTT Features)
  std::vector<cv::KeyPoint> kp1;
  cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
  detector->detect(img1, kp1);

  // now lets track these key points in the second image
  // first use single level LK in the validation picture
  std::vector<cv::KeyPoint> kp2_single;
  std::vector<bool> success_single;
  OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

  // then use multi-level
  std::vector<cv::KeyPoint> kp2_multi;
  std::vector<bool> success_multi;
  auto t1 = std::chrono::steady_clock::now();
  OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
  auto t2 = std::chrono::steady_clock::now();
  auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "Optical flow by Gauss-Newton cost time: " << time_ms << "ms" << std::endl;

  // use opencv's flow for comparison
  std::vector<cv::Point2f> pt1, pt2;
  for (auto &kp : kp1) pt1.push_back(kp.pt);
  std::vector<uchar> status;
  std::vector<float> error;
  t1 = std::chrono::steady_clock::now();
  cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
  t2 = std::chrono::steady_clock::now();
  time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "Optical flow by OpenCV cost time: " << time_ms << "ms" << std::endl;

  // plot the differences of the two methods
  cv::Mat img2_single, img2_multi, img2_opencv;
  cv::cvtColor(img2, img2_single, cv::COLOR_GRAY2BGR);
  cv::cvtColor(img2, img2_multi, cv::COLOR_GRAY2BGR);
  cv::cvtColor(img2, img2_opencv, cv::COLOR_GRAY2BGR);
  for (int i = 0; i < kp2_single.size(); ++i) {
    if (success_single[i]) {
      cv::KeyPoint &kp = kp2_single[i];
      cv::circle(img2_single, kp.pt, 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_single, kp1[i].pt, kp.pt, cv::Scalar(0, 250, 0));
    }
  }

std::cout << kp2_multi.size() << std::endl;
  for (int i = 0; i < kp2_multi.size(); ++i) {
    if (success_multi[i]) {
      cv::KeyPoint &kp = kp2_multi[i];
      std::cout << kp.pt << std::endl;
      cv::circle(img2_multi, kp.pt, 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_multi, kp1[i].pt, kp.pt, cv::Scalar(0, 250, 0));
    }
  }

  for (int i = 0; i < pt2.size(); ++i) {
    if (status[i]) {
      cv::circle(img2_opencv, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_opencv, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    }
  }

  cv::imshow("single level LK", img2_single);
  cv::imshow("multi level LK", img2_multi);
  cv::imshow("opencv LK", img2_opencv);
  cv::waitKey(0);

  return 0;
}
