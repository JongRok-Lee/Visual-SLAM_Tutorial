#include <iostream>
#include <vector>
#include "../common.hpp"

int main(int argc, char **argv) {
  if (argc != 3) {
    throw std::invalid_argument("usage: pose_estimation_2d2d img1 img2");
  }

  // Fetch images
  cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR),
          img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
  assert(img1.data && img2.data && "Can not load images!");

  // Camera Intrinsics TUM Freiburg2
  cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(img1, img2, keypoints1, keypoints2, matches);
  std::cout << "In total, we get " << matches.size() << " set of feature points" << std::endl;

  // Estimate the Motion between two frames
  cv::Mat R, t;
  pose_estimation_2d2d(keypoints1, keypoints2, matches, K, R, t);

  // Check E=t^R*scale
  double t1 = t.at<double>(0, 0);
  double t2 = t.at<double>(1, 0);
  double t3 = t.at<double>(2, 0);
  cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t3, t2,
                                            t3, 0, -t1,
                                            -t2, t1, 0);
  std::cout << "t^R=" << std::endl << t_x * R << std::endl;

  // Check Epipolar Constraint
  for (auto m : matches) {
    cv::Point2d pt1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
    cv::Point2d pt2 = pixel2cam(keypoints2[m.trainIdx].pt, K);

    cv::Mat y1 = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);
    cv::Mat y2 = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);
    cv::Mat d = y2.t() * t_x * R * y1;
    std::cout << "Epipolar constraint = " << d << std::endl;
  }

  return 0;
}
