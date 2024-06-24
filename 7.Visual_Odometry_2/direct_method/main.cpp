#include "direct_method.hpp"

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cout << "Usage: direct_method path_to_left_img path_to_disparity_img path_to_imgs" << std::endl;
    return 1;
  }

  cv::Mat left_img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat disparity_img = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  std::experimental::filesystem::path img_dir_path(argv[3]);
  std::vector<std::string> img_files;
  for (const auto& entry : std::experimental::filesystem::directory_iterator(img_dir_path)) {
    img_files.push_back(entry.path().string());
  }
  std::sort(img_files.begin(), img_files.end());

  // Camera intrinsics
  const double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  // baseline
  const double baseline = 0.573;

  // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
  cv::RNG rng;
  int nPoints = 2000;
  int boarder = 20;
  VecVector2d pixels_ref;
  std::vector<double> depth_ref;

  // generate pixels in ref and load depth data
  for (int i = 0; i < nPoints; ++i) {
    int x = rng.uniform(boarder, left_img.cols - boarder);
    int y = rng.uniform(boarder, left_img.rows - boarder);
    int disparity = disparity_img.at<uchar>(y, x);
    double depth = fx * baseline / disparity;
    depth_ref.push_back(depth);
    pixels_ref.push_back(Eigen::Vector2d(x, y));
  }

  // estimate the camera motion
  Sophus::SE3d T_cur_ref;

  // loop images
  for (const auto& path : img_files) {
    cv::Mat cur_img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    // DirectPoseEstimationSingleLayer(left_img, cur_img, pixels_ref, depth_ref, T_cur_ref);
    DirectPoseEstimationMultiLayer(left_img, cur_img, pixels_ref, depth_ref, T_cur_ref);
  }

  return 0;
}