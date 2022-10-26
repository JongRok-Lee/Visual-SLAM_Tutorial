#include <iostream>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <cmath>
#include <chrono>
#include "g2o/core/g2o_core_api.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

// Vertex: 3D vector
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Override the reset function
        virtual void setToOriginImpl() override {
            _estimate << 0, 0, 0;
        }

        // Override the plus function, just plain vector addition
        virtual void oplusImpl(const double *update) override {
            _estimate += Eigen::Vector3d(update);
        }

        // The dummy read / write function
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
};

// Edge: 1D error term, connected to exactly one vertex
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CurveFittingEdge(double _x) : BaseUnaryEdge(), x(_x) {}

        // Define the error term computation
        virtual void computeError() override {
            const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
            const Eigen::Vector3d abc = v->estimate();
            
            _error(0,0) = _measurement - std::exp(abc(0, 0) * x * x + abc(1, 0) * x + abc(2, 0));
        }

        // The jacobian
        virtual void linearizeOplus() override {
            const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
            const Eigen::Vector3d abc = v->estimate();
            double y = std::exp(abc(0, 0) * x * x + abc(1, 0) * x + abc(2, 0));
            _jacobianOplusXi[0] = -x * x * y;
            _jacobianOplusXi[1] = -x * y;
            _jacobianOplusXi[2] = -y;
        }

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    public:
        double x;           // X data, note y is given in measurement
};

int main(int argc, char **argv) {
    // ... We omit the data sampling code, same as before
    double ar = 1.0, br = 2.0, cr = 1.0;        // Ground-Truth values
    double ae = 2.0, be = -1.0, ce = 5.0;       // Initial values
    int N = 100;                                // Number of data points
    double w_sigma   = 1.0;                     // Sigma of the noise
    double inv_simga = 1.0 / w_sigma;
    cv::RNG rng;                                // OpenCV Random numver generator

    std::vector<double> x_data, y_data;         // Data points
    for (size_t i = 0; i < N; ++i) {
        double x = i / 100.0;
        double y = std::exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    double abc[3] = {ae, be, ce};

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;             // Block Solver
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;   // Linear Solver

    // Choose the optimization method from GN, LM, DogLeg
    // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // auto solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    auto solver = new g2o::OptimizationAlgorithmDogleg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;         // graph optimizer
    optimizer.setAlgorithm(solver);         // Set the algorithm
    optimizer.setVerbose(true);             // Print the results

    // Add vertex
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // Add edges
    for (int i = 0; i < N; ++i) {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);                                                                  // Connect to the vertex
        edge->setMeasurement(y_data[i]);                                                        // measurement
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (w_sigma * w_sigma));    // Set the information matrix
        optimizer.addEdge(edge);
    }

    // Carry out the optimization
    std::cout << "Start optimization" << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Solve time cost = " << time_used.count() << " seconds." << std::endl;

    // Print the results
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "Estimated model = " << abc_estimate.transpose() << std::endl;

    return 0;
}