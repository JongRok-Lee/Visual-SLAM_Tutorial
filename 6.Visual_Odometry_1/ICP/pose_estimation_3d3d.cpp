#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <sophus/se3.hpp>

#include "../common.hpp"

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}
};

/// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

  virtual void computeError() override {
    const VertexPose *pose = static_cast<const VertexPose *> ( _vertices[0] );
    _error = _measurement - pose->estimate() * _point;
  }

  virtual void linearizeOplus() override {
    VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = pose->estimate();
    Eigen::Vector3d xyz_trans = T * _point;
    _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
  }

  bool read(std::istream &in) {}

  bool write(std::ostream &out) const {}

protected:
  Eigen::Vector3d _point;
};

void pose_estimation_3d3d(const std::vector<cv::Point3f> &pts1,
                          const std::vector<cv::Point3f> &pts2,
                          cv::Mat &R, cv::Mat &t);
void bundleAdjustment(const std::vector<cv::Point3f> &pts1,
                      const std::vector<cv::Point3f> &pts2,
                      cv::Mat &R, cv::Mat &t);

int main(int argc, char* argv[]) {
  if (argc != 5) {
    throw std::invalid_argument("usage: pose_estimation_3d3d img1 img2 depth1 depth2");
  }

  // Image Load
  cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR);
  cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
  assert(img1.data && img2.data && "Can not load images!");

  // Camera Intrinsics TUM Freiburg2
  cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  // Kepoint match
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(img1, img2, keypoints1, keypoints2, matches);
  std::cout << "In total, we get " << matches.size() << " set of feature points" << std::endl;

  // Depth image Load
  cv::Mat depth1 = cv::imread(argv[3], cv::IMREAD_ANYDEPTH);
  cv::Mat depth2 = cv::imread(argv[4], cv::IMREAD_ANYDEPTH);
  std::vector<cv::Point3f> pts1, pts2;

  for (cv::DMatch m : matches) {
    uint16_t d1 = depth1.ptr<uint16_t>(static_cast<int>(keypoints1[m.queryIdx].pt.y))[static_cast<int>(keypoints1[m.queryIdx].pt.x)];
    uint16_t d2 = depth2.ptr<uint16_t>(static_cast<int>(keypoints2[m.trainIdx].pt.y))[static_cast<int>(keypoints2[m.trainIdx].pt.x)];
    if (d1 == 0 || d2 == 0) continue;

    cv::Point2d p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
    cv::Point2d p2 = pixel2cam(keypoints2[m.trainIdx].pt, K);
    float dd1 = static_cast<float>(d1) / 5000.f;
    float dd2 = static_cast<float>(d2) / 5000.f;
    pts1.push_back(cv::Point3f(p1.x * dd1, p1.y * dd1, dd1));
    pts2.push_back(cv::Point3f(p2.x * dd2, p2.y * dd2, dd2));
  }
  std::cout << "3D-3D pairs: " << pts1.size() << std::endl;

  cv::Mat R, t;
  pose_estimation_3d3d(pts1, pts2, R, t);
  std::cout << "ICP via SVD results: " << std::endl;
  std::cout << "R = " << R << std::endl;
  std::cout << "t = " << t << std::endl;
  std::cout << "R_inv = " << R.t() << std::endl;
  std::cout << "t_inv = " << -R.t() * t << std::endl;

  std::cout << "calling bundle adjustment" << std::endl;

  bundleAdjustment(pts1, pts2, R, t);

  // verify p1 = R * p2 + t
  for (int i = 0; i < 5; i++) {
    std::cout << "p1 = " << pts1[i] << std::endl;
    std::cout << "p2 = " << pts2[i] << std::endl;
    std::cout << "(R*p2+t) = " <<
         R * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t
         << std::endl;
    std::cout << std::endl;
  }

}

void pose_estimation_3d3d(const std::vector<cv::Point3f> &pts1,
                          const std::vector<cv::Point3f> &pts2,
                          cv::Mat &R, cv::Mat &t)
{
  // Calculate Center of Mass
  cv::Point3f p1, p2;
  int N = pts1.size();
  std::cout << "N: " << N << std::endl;
  for (size_t i = 0; i < N; ++i) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 /= N;
  p2 /= N;

  std::vector<cv::Point3f> q1(N), q2(N);
  for (size_t i = 0; i < N; ++i) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // Comput q1*q2.T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < N; ++i) {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  std::cout << "W = " << W << std::endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  std::cout << "U = " << U << std::endl;
  std::cout << "V = " << V << std::endl;

  Eigen::Matrix3d R_ = U * V.transpose();
  if (R_.determinant() < 0) {
    R_ = -R_;
  }

  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // Convert to cv::Mat
  R = cv::Mat_<double>(3, 3) <<
    R_(0, 0), R_(0, 1), R_(0, 2),
    R_(1, 0), R_(1, 1), R_(1, 2),
    R_(2, 0), R_(2, 1), R_(2, 2);
  t = cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0);
}

void bundleAdjustment(const std::vector<cv::Point3f> &pts1,
                      const std::vector<cv::Point3f> &pts2,
                      cv::Mat &R, cv::Mat &t)
{
  typedef g2o::BlockSolverX BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
  
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // vertex
  VertexPose *pose = new VertexPose(); // camera pose
  pose->setId(0);
  pose->setEstimate(Sophus::SE3d());
  optimizer.addVertex(pose);

  // edges
  for (size_t i = 0; i < pts1.size(); i++) {
    EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
      Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
    edge->setVertex(0, pose);
    edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge);
  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1);
  std::cout << "Optimization costs time: " << time.count() << " ms.\n";
  
  std::cout << std::endl << "After Optimization\n";
  std::cout << "T=\n" << pose->estimate().matrix() << std::endl;

  // Conver to cv::Mat
  Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
  Eigen::Vector3d t_ = pose->estimate().translation();

  R = cv::Mat_<double>(3, 3) <<
    R_(0, 0), R_(0, 1), R_(0, 2),
    R_(1, 0), R_(1, 1), R_(1, 2),
    R_(2, 0), R_(2, 1), R_(2, 2);
  t = cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0);
}
