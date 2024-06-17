#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

int main(int argc, char** argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;        // Ground-Truth values
    double ae = 2.0, be = -1.0, ce = 5.0;       // Initial values
    int N = 100;                                // Number of data points
    double w_sigma   = 1.0;                     // Sigma of the noise
    double inv_simga = 1.0 / w_sigma;
    cv::RNG rng;                                // OpenCV Random number generator
    std::cout.precision(3);

    std::vector<double> x_data, y_data;         // Data points
    for (size_t i = 0; i < N; ++i) {
        double x = i / 100.0;
        double y = std::exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    // Start Gauss-Newton iterations
    int iterNumber = 200;
    double cost = 0, lastCost = 0;
    double lambda = 1e-4;                               // Lambda for LM
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t iter = 0; iter < iterNumber; ++iter) {
        cost = 0;
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();        // Hessian = J * J^T
        Eigen::Matrix3d diag_H = Eigen::Matrix3d::Zero();   // diag(Hessian)
        Eigen::Vector3d b = Eigen::Vector3d::Zero();        // Bias
        Eigen::Vector3d J;
        for (size_t i = 0; i < N; ++i) {
            double xi = x_data[i], yi = y_data[i];          // The i-th data
            double y_hat = std::exp(ae*xi*xi + be*xi + ce); // Estimated y
            double error = yi - y_hat;
            
            J(0,0) = -xi * xi * y_hat;                      // derror/da
            J(1,0) = -xi * y_hat;                           // derror/db
            J(2,0) = -y_hat;                                // derror/dc

            H += J * J.transpose();
            diag_H += H.diagonal().asDiagonal();
            b += -error * J;
            cost += error * error;
        }

        // Sovle (H+lambda*diag(H))x = b
        Eigen::Vector3d dx = (H + lambda * diag_H).ldlt().solve(b);
        if (isnan(dx[0])) {
            std::cout << "Result is NaN!\n";
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // Cost increase, Update is not good.
            std::cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << std::endl;
            break;
        }

        // Trust region
        double new_cost = 0;
        double a_temp = ae + dx(0,0);
        double b_temp = be + dx(1,0);
        double c_temp = ce + dx(2,0);

        for (size_t i = 0; i < N; ++i) {
            double xi = x_data[i], yi = y_data[i];          // The i-th data
            double y_hat = std::exp(a_temp*xi*xi + b_temp*xi + c_temp); // Estimated y
            double error = yi - y_hat;
            new_cost += error * error;
        }
        
        if (new_cost < cost) {
            lambda /= 10;
            ae += dx(0,0);
            be += dx(1,0);
            ce += dx(2,0);
        } else {
            lambda *= 10;
        }
        std::cout << "lambda: " << lambda << std::endl;
        lastCost = cost;
        std::cout << iter << ". total cost: " << cost << ", \t\t\tupdate: " << dx.transpose() << "\t\t\testimated params: " << ae << "," << be << "," << ce << std::endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;
    std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;

    return 0;
}   