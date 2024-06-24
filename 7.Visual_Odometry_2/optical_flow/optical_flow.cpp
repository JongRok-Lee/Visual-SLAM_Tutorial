#include "optical_flow.hpp"

OpticalFlowTracker::OpticalFlowTracker(const cv::Mat &img1, const cv::Mat &img2,
                                       const std::vector<cv::KeyPoint> &kp1,
                                       std::vector<cv::KeyPoint> &kp2,
                                       std::vector<bool> &success,
                                       bool inverse,
                                       bool has_initial) :
                                       img1_(img1), img2_(img2), kp1_(kp1), kp2_(kp2),
                                       success_(success), inverse_(inverse),
                                       has_initial_(has_initial) {}

void OpticalFlowTracker::calculateOpticalFlow(const cv::Range &range) {
  // parameters
  int half_patch_size = 4;
  int iterations = 10;
  for (size_t i = range.start; i < range.end; i++) {
    auto kp = kp1_[i];
    double dx = 0, dy = 0; // dx,dy need to be estimated
    if (has_initial_) {
      dx = kp2_[i].pt.x - kp.pt.x;
      dy = kp2_[i].pt.y - kp.pt.y;
    }

    double cost = 0, lastCost = 0;
    bool succ = true; // indicate if this point succeeded

    // Gauss-Newton iterations
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();    // hessian
    Eigen::Vector2d b = Eigen::Vector2d::Zero();    // bias
    Eigen::Vector2d J;  // jacobian
    for (int iter = 0; iter < iterations; iter++) {
      if (inverse_ == false) {
        H = Eigen::Matrix2d::Zero();
        b = Eigen::Vector2d::Zero();
      } else {
        // only reset b
        b = Eigen::Vector2d::Zero();
      }

      cost = 0;

      // compute cost and jacobian
      for (int x = -half_patch_size; x < half_patch_size; x++) {
        for (int y = -half_patch_size; y < half_patch_size; y++) {
          double error = GetPixelValue(img1_, kp.pt.x + x, kp.pt.y + y) -
                          GetPixelValue(img2_, kp.pt.x + x + dx, kp.pt.y + y + dy);;  // Jacobian
          if (inverse_ == false) {
            J = -1.0 * Eigen::Vector2d(
              0.5 * (GetPixelValue(img2_, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                      GetPixelValue(img2_, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
              0.5 * (GetPixelValue(img2_, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                      GetPixelValue(img2_, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
            );
          } else if (iter == 0) {
            // in inverse mode, J keeps same for all iterations
            // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
            J = -1.0 * Eigen::Vector2d(
              0.5 * (GetPixelValue(img1_, kp.pt.x + x + 1, kp.pt.y + y) -
                      GetPixelValue(img1_, kp.pt.x + x - 1, kp.pt.y + y)),
              0.5 * (GetPixelValue(img1_, kp.pt.x + x, kp.pt.y + y + 1) -
                      GetPixelValue(img1_, kp.pt.x + x, kp.pt.y + y - 1))
            );
          }
          // compute H, b and set cost;
          b += -error * J;
          cost += error * error;
          if (inverse_ == false || iter == 0) {
            // also update H
            H += J * J.transpose();
          }
        }
      }

      // compute update
      Eigen::Vector2d update = H.ldlt().solve(b);

      if (std::isnan(update[0])) {
        // sometimes occurred when we have a black or white patch and H is irreversible
        std::cout << "update is nan" << std::endl;
        succ = false;
        break;
      }

      if (iter > 0 && cost > lastCost) {
        break;
      }

      // update dx, dy
      dx += update[0];
      dy += update[1];
      lastCost = cost;
      succ = true;

      if (update.norm() < 1e-2) {
        // converge
        break;
      }
    }

    success_[i] = succ;

    // set kp2
    kp2_[i].pt = kp.pt + cv::Point2f(dx, dy);
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

void OpticalFlowSingleLevel(const cv::Mat &img1, const cv::Mat &img2,
                            const std::vector<cv::KeyPoint> &kp1,
                            std::vector<cv::KeyPoint> &kp2,
                            std::vector<bool> &success,
                            bool inverse, bool has_initial)
{
  kp2.resize(kp1.size());
  success.resize(kp1.size());
  OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
  parallel_for_(cv::Range(0, kp1.size()),
                std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, std::placeholders::_1));
  // for (auto kp : kp2) {
  //   std::cout << kp.pt << std::endl;
  // }
}

void OpticalFlowMultiLevel(const cv::Mat &img1, const cv::Mat &img2,
                           const std::vector<cv::KeyPoint> &kp1,
                           std::vector<cv::KeyPoint> &kp2,
                           std::vector<bool> &success,
                           bool inverse)
{
  int pyramids = 4;
  double pyramid_scale = 0.5;
  double scales[] = {1.0, 0.5, 0.25, 0.125};

  // build pyramid images
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::vector<cv::Mat> pyr1, pyr2; // image pyramids
  for (int i = 0; i < pyramids; ++i) {
    if (i == 0) {
      pyr1.push_back(img1);
      pyr2.push_back(img2);
    } else {
      cv::Mat img1_pyr, img2_pyr;
      cv::resize(pyr1[i - 1], img1_pyr, cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
      cv::resize(pyr2[i - 1], img2_pyr, cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
      pyr1.push_back(img1_pyr);
      pyr2.push_back(img2_pyr);
    }
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "Pyramid construction time: " << time_ms << "ms" << std::endl;

  // coarse-to-fine LK tracking in pyramids
  std::vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
  for (auto &kp : kp1) {
    cv::KeyPoint kp_top = kp;
    kp_top.pt *= scales[pyramids - 1];
    kp1_pyr.push_back(kp_top);
    kp2_pyr.push_back(kp_top);
  }

  for (int level = pyramids - 1; level >= 0; --level) {
    // from coarse to fine
    success.clear();
    t1 = std::chrono::steady_clock::now();
    OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);
    t2 = std::chrono::steady_clock::now();
    time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "track pyr " << level << " cost time: " << time_ms << "ms" << std::endl;

    if (level > 0) {
      for (cv::KeyPoint &kp : kp1_pyr) {
        kp.pt /= pyramid_scale;
      }
      for (cv::KeyPoint &kp : kp2_pyr) {
        kp.pt /= pyramid_scale;
      }
    }
  }

  for (auto &kp : kp2_pyr) {
    kp2.push_back(kp);
  }
}