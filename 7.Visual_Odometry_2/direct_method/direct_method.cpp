#include "direct_method.hpp"

// Camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;

JacobianAccumulator::JacobianAccumulator(
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  const std::vector<double> depth_ref,
  Sophus::SE3d &T21) :
  img1_(img1), img2_(img2), px_ref_(px_ref), depth_ref_(depth_ref), T21_(T21)
  { projection_ = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0)); }

Matrix6d JacobianAccumulator::hessian() const { return H_;}

Vector6d JacobianAccumulator::bias() const { return b_;}

double JacobianAccumulator::cost_func() const { return cost_;}

VecVector2d JacobianAccumulator::projected_points() const { return projection_;}
void JacobianAccumulator::reset()
{
  H_ = Matrix6d::Zero();
  b_ = Vector6d::Zero();
  cost_ = 0.0;
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range &range)
{
  // parameters
  const int half_patch_size = 1;
  int cnt_good = 0;
  Matrix6d hessian = Matrix6d::Zero();
  Vector6d bias = Vector6d::Zero();
  double cost_temp = 0.0;

  for (size_t i = range.start; i < range.end; ++i) {
    // compute the projection in the second image
    Eigen::Vector3d point_ref =
      depth_ref_[i] * Eigen::Vector3d((px_ref_[i][0] - cx) / fx, (px_ref_[i][1] - cy) / fy, 1);
    Eigen::Vector3d point_cur = T21_ * point_ref;
    if (point_cur[2] < 0) continue;

    float u = fx * point_cur[0] / point_cur[2] + cx;
    float v = fy * point_cur[1] / point_cur[2] + cy;
    if (u < half_patch_size || u > img2_.cols - half_patch_size ||
        v < half_patch_size || v > img2_.rows - half_patch_size) continue;

    projection_[i] = Eigen::Vector2d(u, v);
    double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
           Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
    cnt_good++;

    // compute Jacobians
    for (int x = -half_patch_size; x <= half_patch_size; ++x) {
      for (int y = -half_patch_size; y <= half_patch_size; ++y) {
        double error = GetPixelValue(img1_, px_ref_[i][0] + x, px_ref_[i][1] + y) -
                       GetPixelValue(img2_, u + x, v + y);
        Matrix26d J_pixel_xi;
        Eigen::Vector2d J_img_pixel;

        J_pixel_xi(0, 0) = fx * Z_inv;
        J_pixel_xi(0, 1) = 0;
        J_pixel_xi(0, 2) = -fx * X * Z2_inv;
        J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
        J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
        J_pixel_xi(0, 5) = -fx * Y * Z_inv;

        J_pixel_xi(1, 0) = 0;
        J_pixel_xi(1, 1) = fy * Z_inv;
        J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
        J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
        J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
        J_pixel_xi(1, 5) = fy * X * Z_inv;

        J_img_pixel = Eigen::Vector2d(
          0.5 * (GetPixelValue(img2_, u + x + 1, v + y) - GetPixelValue(img2_, u + x - 1, v + y)),
          0.5 * (GetPixelValue(img2_, u + x, v + y + 1) - GetPixelValue(img2_, u + x, v + y - 1))
        );

        // total jacobian
        Vector6d J = -(J_img_pixel.transpose() * J_pixel_xi).transpose();

        hessian += J * J.transpose();
        bias += -error * J;
        cost_temp += error * error;
      }
    }
  }

  if (cnt_good) {
    std::unique_lock<std::mutex> lock(hessian_mutex_);
    H_ += hessian;
    b_ += bias;
    cost_ += cost_temp / cnt_good;
  }
}

float GetPixelValue(const cv::Mat &img, float x, float y) {
  // boundary check
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x >= img.cols - 1) x = img.cols - 2;
  if (y >= img.rows - 1) y = img.rows - 2;

  float xx = x - floor(x);
  float yy = y - floor(y);
  int x_a1 = std::min(img.cols - 1, int(x) + 1);
  int y_a1 = std::min(img.rows - 1, int(y) + 1);

  return (1 - xx) * (1 - yy) * img.at<uchar>(y, x)
  + xx * (1 - yy) * img.at<uchar>(y, x_a1)
  + (1 - xx) * yy * img.at<uchar>(y_a1, x)
  + xx * yy * img.at<uchar>(y_a1, x_a1);
}

void DirectPoseEstimationSingleLayer(
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  const std::vector<double> depth_ref,
  Sophus::SE3d &T21)
{
  const int iterations = 10;
  double cost = 0, lastCost = 0;
  auto t1 = std::chrono::steady_clock::now();
  JacobianAccumulator jaco_acc(img1, img2, px_ref, depth_ref, T21);

  for (int iter = 0; iter < iterations; ++iter) {
    jaco_acc.reset();
    cv::parallel_for_(
      cv::Range(0, px_ref.size()),
      std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_acc, std::placeholders::_1)
    );
    Matrix6d H = jaco_acc.hessian();
    Vector6d b = jaco_acc.bias();

    // solve update and put it into estimation
    Vector6d update = H.ldlt().solve(b);
    T21 = Sophus::SE3d::exp(update) * T21;
    cost = jaco_acc.cost_func();

    if (std::isnan(update[0])) {
      std::cout << "update is nan" << std::endl;
      break;
    }
    if (iter > 0 && cost > lastCost) {
      std::cout << "cost increased: " << cost << ", " << lastCost << std::endl;
      break;
    }
    if (update.norm() < 1e-3) {
      // converge
      break;
    }

    lastCost = cost;
    std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << std::endl;
  }

  auto t2 = std::chrono::steady_clock::now();
  auto time_used = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "T21 = \n" << T21.matrix() << std::endl;
  std::cout << "direct method costs time: " << time_used << " milliseconds." << std::endl;

  // plot the projected pixels
  cv::Mat img2_show;
  cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
  VecVector2d projected_points = jaco_acc.projected_points();
  for (size_t i = 0; i < px_ref.size(); ++i) {
    if (projected_points[i][0] > 0 && projected_points[i][1] > 0) {
      cv::circle(img2_show, cv::Point2f(projected_points[i][0], projected_points[i][1]), 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_show, cv::Point2f(px_ref[i][0], px_ref[i][1]),
               cv::Point2f(projected_points[i][0], projected_points[i][1]), cv::Scalar(0, 250, 0));
    }
  }
  cv::imshow("result", img2_show);
  cv::waitKey(0);
}

void DirectPoseEstimationMultiLayer(
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  const std::vector<double> depth_ref,
  Sophus::SE3d &T21) {

  // parameters
  int pyramids = 4;
  double pyramid_scale = 0.5;
  double scales[] = {1.0, 0.5, 0.25, 0.125};

  // create pyramids
  std::vector<cv::Mat> pyr1, pyr2; // image pyramids
  for (int i = 0; i < pyramids; i++) {
    if (i == 0) {
      pyr1.push_back(img1);
      pyr2.push_back(img2);
    } else {
      cv::Mat img1_pyr, img2_pyr;
      cv::resize(pyr1[i - 1], img1_pyr,
                  cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
      cv::resize(pyr2[i - 1], img2_pyr,
                  cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
      pyr1.push_back(img1_pyr);
      pyr2.push_back(img2_pyr);
    }
  }

  double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
  for (int level = pyramids - 1; level >= 0; level--) {
    VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
    for (auto &px: px_ref) {
        px_ref_pyr.push_back(scales[level] * px);
    }

    // scale fx, fy, cx, cy in different pyramid levels
    fx = fxG * scales[level];
    fy = fyG * scales[level];
    cx = cxG * scales[level];
    cy = cyG * scales[level];
    DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
  }

}