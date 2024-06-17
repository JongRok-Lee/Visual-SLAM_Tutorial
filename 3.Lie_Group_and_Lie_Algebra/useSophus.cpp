#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

/// This program demonstrates the basic usage of Sophus

// Lie Algebra is 6d vector, we give a typedef
namespace Eigen {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

int main(int argc, char **argv) {

  // Rotation matrix with 90 degrees along Z axis
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  // or quaternion
  Eigen::Quaterniond q(R);
  Sophus::SO3d SO3_R(R);                  // Sophus::SO3d can be constructed from rotation matrix
  Sophus::SO3d SO3_q(q);                  // or quaternion

  // they are equivalent of course
  std::cout << "Rotation matrix:\n" << R << std::endl;
  std::cout << "SO(3) from matrix:\n" << SO3_R.matrix() << std::endl;
  std::cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << std::endl;
  std::cout << "they are equal" << std::endl << std::endl << std::endl ;

  // Use logarithmic map to get the Lie algebra
  Eigen::Vector3d so3 = SO3_R.log();
  std::cout << "so(3) = " << so3.transpose() << std::endl;

  // hat is from vector to skew−symmetric matrix
  Eigen::Matrix3d so3_hat = Sophus::SO3d::hat(so3);
  std::cout << "so(3) hat=\n" << so3_hat << std::endl;

  // inversely from matrix to vector
  Eigen::Vector3d so3_vee = Sophus::SO3d::vee(so3_hat);
  std::cout << "so(3) hat vee= " << so3_vee.transpose() << std::endl;

  // Use Exponential map to get the Lie Group
  Sophus::SO3d SO3_exp = Sophus::SO3d::exp(so3);
  std::cout << "SO(3) with Exponential map = " << std::endl << SO3_exp.matrix() << std::endl;

  // update by perturbation model
  Eigen::Vector3d update_so3(1e-4, 0, 0); // this is a small update
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  std::cout << "SO3 updated = \n" << SO3_updated.matrix() << std::endl;

  std::cout << "*******************************" << std::endl;
  // Similar for SE(3)
  Eigen::Vector3d t(1, 0, 0);             // translation 1 along X
  Sophus::SE3d SE3_Rt(R, t);              // construction SE3 from R,t
  Sophus::SE3d SE3_qt(q, t);              // or q,t

  std::cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << std::endl;
  std::cout << "SE3 from q,t= \n" << SE3_qt.matrix() << std::endl;

  // Use logarithmic map to get the Lie algebra
  Eigen::Vector6d se3 = SE3_Rt.log();
  std::cout << "se3 = " << se3.transpose() << std::endl;

  // The output shows Sophus puts the translation at first in se(3), then rotation.
  // Save as SO(3) wehave hat and vee
  Eigen::Matrix4d se3_hat = Sophus::SE3d::hat(se3);
  Eigen::Vector6d se3_vee = Sophus::SE3d::vee(se3_hat);
  std::cout << "se3 hat = \n" << se3_hat << std::endl;
  std::cout << "se3 hat vee = " << se3_vee.transpose() << std::endl;

  // Use Exponential map to get the Lie Group
  Sophus::SE3d SE3_exp = Sophus::SE3d::exp(se3);
  std::cout << "SE(3) with Exponential map = " << std::endl << SE3_exp.matrix() << std::endl;

  // Finally the update
  Eigen::Vector6d update_se3;
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;

  return 0;
}
