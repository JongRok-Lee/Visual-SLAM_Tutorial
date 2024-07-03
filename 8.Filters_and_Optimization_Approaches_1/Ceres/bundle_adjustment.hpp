#ifndef _BUNDLE_ADJUSTMENT_HPP_
#define _BUNDLE_ADJUSTMENT_HPP_

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../rotation.h"
#include "../random.h"

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

class BALProblem {
public:
  BALProblem(const std::string &filename, bool use_quaternions = false);
  ~BALProblem();
  void WriteToPLYFile(const std::string &filename) const;
  void Normalize();
  void Perturb(const double rotation_sigma, const double translation_sigma, const double point_sigma);
  void CameraToAngelAxisAndCenter(const double *camera, double *angle_axis, double *center) const;
  void AngleAxisAndCenterToCamera(const double *angle_axis, const double *center, double *camera) const;

  int camera_block_size() const { return use_quaternions_ ? 10 : 9; }
  int point_block_size() const { return 3; }

  int num_cameras() const { return num_cameras_; }
  int num_points() const { return num_points_; }
  int num_observations() const { return num_observations_; }
  int num_parameters() const { return num_parameters_; }

  const int *point_index() const { return point_index_; }
  const int *camera_index() const { return camera_index_; }

  const double *observations() const { return observations_; }
  const double *parameters() const { return parameters_; }
  const double *cameras() const { return parameters_; }
  const double *points() const { return parameters_ + camera_block_size() * num_cameras_; }

  double *mutable_cameras() { return parameters_; }
  double *mutable_points() { return parameters_ + camera_block_size() * num_cameras_; }

private:
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  bool use_quaternions_;

  int *point_index_;
  int *camera_index_;
  double *observations_;
  double *parameters_;
};

#endif // _BUNDLE_ADJUSTMENT_HPP_
