#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/geometry.hpp>

#include "../common.hpp"

namespace Eigen {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    throw std::invalid_argument("usage: pose_estimation_3d2d img1 img2 depth1");
  }

  // Fetch RGB images
  cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR),
          img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
  assert(img1.data && img2.data && "Can not load images!");

  // Camera Intrinsics TUM Freiburg2
  cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(img1, img2, keypoints1, keypoints2, matches);

  cv::Mat imgDepth1 = cv::imread(argv[3], cv::IMREAD_ANYDEPTH);

  std::vector<cv::Point3f> pts3D;
  std::vector<cv::Point2f> pts2D;
  for (cv::DMatch m : matches) {
    uint16_t d = imgDepth1.ptr<uint16_t>(static_cast<int>(keypoints1[m.queryIdx].pt.y))[static_cast<int>(keypoints1[m.queryIdx].pt.x)];
    if (d == 0) continue;

    float dd = d / 5000.f;
    cv::Point2f p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
    pts3D.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
    pts2D.push_back(keypoints2[m.trainIdx].pt);
  }

  cv::Mat r, R, t;
  cv::solvePnP(pts3D, pts2D, K, cv::noArray(), r, t, false);
  cv::Rodrigues(r, R);
  Eigen::Matrix4d pose;
  pose << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
          R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
          R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
          0, 0, 0, 1;

  std::vector<Eigen::Vector3d> ptsEigen3D;
  std::vector<Eigen::Vector3d> ptsEigen2D;
  for (int i = 0; i < pts3D.size(); ++i) {
    ptsEigen3D.push_back(Eigen::Vector3d(pts3D[i].x, pts3D[i].y, pts3D[i].z));
    ptsEigen2D.push_back(Eigen::Vector2d(pts2D[i].x, pts2D[i].y));
  }

}