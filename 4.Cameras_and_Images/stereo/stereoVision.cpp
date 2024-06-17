#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <chrono>
#include <thread>

namespace Eigen {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

void showPointCloud(const std::vector<Eigen::Vector6d> &pointcloud);

int main(int argc, char *argv[]) {

  std::string left_file = argv[1];
  std::string right_file = argv[2];

  // Intrinsic Parameters
  double fx = 721.538, fy = 721.538, cx = 609.5593, cy = 172.8540;
  // baseline
  double b = 0.54;

  cv::Mat left = cv::imread(left_file, cv::IMREAD_ANYCOLOR);
  cv::Mat right = cv::imread(right_file, cv::IMREAD_ANYCOLOR);
  int rows = left.rows, cols = left.cols;

  std::cout << "Image cols: " << cols
            << " , rows: " << rows
            << " , channels: " << left.channels() << std::endl;

  // SGBM is senstive to parameters
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
      0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

  // Compute the point cloud
  std::vector<Eigen::Vector6d> pointcloud;

  // Chagne v++ and u++ to v+=2, u+=2 if yout machine is slow to get a sparser cloud
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

      // The dimensions are xyz and bgr.
      Eigen::Vector6d point;

      // Compute the depth from disparity
      // u = fx*x + cx, v = fy*y + cy
      // z = fx * b / (uL - uR)
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      double depth = fx * b / (disparity.at<float>(v, u));
      point[0] = x * depth;
      point[1] = y * depth;
      point[2] = depth;

      // Get the BGR values
      cv::Vec3b bgr = left.at<cv::Vec3b>(v, u);
      point[3] = static_cast<double>(bgr[0])/255;
      point[4] = static_cast<double>(bgr[1])/255;
      point[5] = static_cast<double>(bgr[2])/255;

      pointcloud.push_back(point);
    }
  }

  cv::imshow("disparity", disparity / 96.0);
  cv::waitKey(0);
  
  // show the point cloud in pangolin
  showPointCloud(pointcloud);
  return 0;
}

void showPointCloud(const std::vector<Eigen::Vector6d> &pointcloud) {

  if (pointcloud.empty()) {
    std::cerr << "Point cloud is empty!" << std::endl;
    return;
  }

  pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p: pointcloud) {
      glColor3f(p[5], p[4], p[3]);
      glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));   // sleep 5 ms
  }
  return;
}