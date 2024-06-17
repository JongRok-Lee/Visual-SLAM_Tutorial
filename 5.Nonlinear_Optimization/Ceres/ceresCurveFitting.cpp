#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <ceres/ceres.h>

// Residual function
struct CURVE_FITTING_COST {

  const double x, y;
  CURVE_FITTING_COST(double _x, double _y) : x(_x), y(_y) {}

  // Implement operator () to compute the error
  template<typename T>
  bool operator()(const T *const abc, T *residual) const {                                // *const abc => Do not change abc!, const T => 
    // residual = y - exp(ax^2 + bx + c)
    residual[0] = T(y) - ceres::exp(abc[0] * T(x) * T(x) + abc[1] * T(x) + abc[2]);
    return true;
  }
};

// CURVE_FITTING_COST(abc, residual);

int main(int argc, char **argv) {
  // Same as before (Gauss-Newton Method)
  double ar = 1.0, br = 2.0, cr = 1.0;        // Ground-Truth values
  double ae = 2.0, be = -1.0, ce = 5.0;       // Initial values
  int N = 100;                                // Number of data points
  double w_sigma   = 1.0;                     // Sigma of the noise
  double inv_simga = 1.0 / w_sigma;
  cv::RNG rng;                                // OpenCV Random number generator

  std::vector<double> x_data, y_data;         // Data points
  for (size_t i = 0; i < N; ++i) {
    double x = i / 100.0;
    double y = std::exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma);
    x_data.push_back(x);
    y_data.push_back(y);
  }

  double abc[3] = {ae, be, ce};

  // Construct the problem in ceres
  ceres::Problem problem;
  for (int i = 0; i < N; ++i) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
      nullptr,
      abc);
  }

  // Set the solver options
  ceres::Solver::Options options;             // Actually there are lots of params can be adjusted
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // Use Cholesky to solve the normal equation
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "Solve time cost = " << time_used.count() << " seconds." << std::endl;

  // Get the outputs
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "Estimated a, b, c = " << abc[0] << ", " << abc[1] << ", " << abc[2] << std::endl;

  return 0;
}

