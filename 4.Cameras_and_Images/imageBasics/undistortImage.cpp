# include <opencv2/opencv.hpp>
# include <string>

int main(int argc, char** argv)
{
  std::string image_file = argv[1];
  /// In this program, we implement the undistortion by ourselves rather than using opencv
  // Distortion Parameters (rad−tan model)
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

  // Intrinsic Parameters
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, cv::IMREAD_ANYCOLOR);
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat::zeros(rows, cols, CV_8UC1);    // the distorted image

  // computate the pixels in the undistorted one
  for (size_t v = 0; v < rows; ++v) {
    for (size_t u = 0; u < cols; ++u) {
      // note we are computing the pixel of (u,v) in the undistorted image
      // according to the rad−tan model, compute the coordinates in the distorted image
      // u = fx*x + cx, v = fy*y + cy
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      double r = std::sqrt(x * x + y * y);

      double x_distorted = x*(1 + k1*r*r + k2*r*r*r*r) + 2*p1*x*y + p2*(r*r + 2*x*x);
      double y_distorted = y*(1 + k1*r*r + k2*r*r*r*r) + 2*p2*x*y + p1*(r*r + 2*y*y);

      int u_distorted = static_cast<int>(fx*x_distorted + cx);
      int v_distorted =  static_cast<int>(fy*y_distorted + cy);

      // Check if the pixel is in the image boarder
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted <= cols && v_distorted <= rows) {
        image_undistort.at<uint8_t>(v, u) = image.at<uint8_t>(v_distorted, u_distorted);
      }
    }
  }

  // show the undistorted image
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey(0);
  return 0;
}