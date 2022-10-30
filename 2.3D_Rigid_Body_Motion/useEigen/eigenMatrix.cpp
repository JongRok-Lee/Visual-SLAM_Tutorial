#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char **argv) {
    // All vectors and matrices in Eigen are Eigen::Matrix, which is a template
    // class. Its first three parameters are: data type, row, column Declare a 2∗3 float matrix
    Eigen::Matrix<float, 2, 3> matrixf_23;

    // At the same time, Eigen provides many built−in types via typedef, but the
    // bottom layer is still Eigen::Matrix. For example, Vector3d is essentially
    // Eigen::Matrix<double, 3, 1>, which is a three−dimensional vector.
    Eigen::Vector3d v_3d;


    // This is the same with float type
    Eigen::Matrix<float, 3, 1> vf_3d;

    // Matrix3d is essentially Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); // initialized to zero

    // If you are not sure about the size of the matrix, you can use a matrix of dynamic size
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // simpler
    Eigen::MatrixXd matrix_x;
    // There are still many types of this kind. We don't list them one by one.

    // Here is the operation of the Eigen matrix
    // input data (initialization)
    matrixf_23 << 1, 2, 3,
                    4, 5, 6;
    // output
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrixf_23 << std::endl;

    // Use () to access elements in the matrix
    std::cout << "print matrix 2x3: " << std::endl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << matrixf_23(i, j) << "\t";
        }
        std::cout << std::endl;
    }

    // We can easily multiply a matrix with a vector (but actually still matrices and matrices)
    v_3d << 3, 2, 1;
    vf_3d << 4, 5, 6;

    // In Eigen you can't mix two different types of matrices, like this is
    // wrong Matrix<double, 2, 1> result_wrong_type = matrixf_23 ∗ v_3d;
    // It should be explicitly converted
    Eigen::Matrix<double, 2, 1> resultd = matrixf_23.cast<double>() * v_3d;
    std::cout << "[1,2,3;4,5,6]*[3,2,1]=" << resultd.transpose() << std::endl;

    Eigen::Matrix<float, 2, 1> resultf = matrixf_23 * vf_3d;
    std::cout << "[1,2,3;4,5,6]*[4,5,6]: " << resultf.transpose() << std::endl;

    // Also you can't misjudge the dimensions of the matrix
    //Try canceling the comments below to see what Eigen will report.
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() ∗ v_3d;

    // some matrix operations
    // The basic operations are not demonstrated, just use +−∗/ operators.
    matrix_33 = Eigen::Matrix3d::Random();                                // Random Number Matrix
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "times 10: \n" << 10 * matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;

    // Eigenvalues
    // Real symmetric matrix can guarantee successful diagonalization
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    // Solving equations
    // We solve the equation of matrix_NN ∗ x = v_Nd
    // The size of N is defined in the previous macro, which is generated by a
    // random number Direct inversion is the most direct, but the amount of
    // inverse operations is large.

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = 
        Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();  // Guarantee semi−positive definite
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    std::chrono::steady_clock::time_point time_stt = std::chrono::steady_clock::now(); // timing
    // Direct inversion
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - time_stt);
    std::cout << "Time of normal inverse is " << time_used.count() << " seconds.\n";
    std::cout << "x = " << x.transpose() << std::endl << std::endl;

    // Usually solved by matrix decomposition, such as QR decomposition, the speed will be much faster
    time_stt = std::chrono::steady_clock::now();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    time_used = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_stt);
    std::cout << "Time of normal inverse is " << time_used.count() << " seconds.\n";
    std::cout << "x = " << x.transpose() << std::endl << std::endl;

    // For positive definite matrices, you can also use cholesky decomposition to solve equations.
    time_stt = std::chrono::steady_clock::now();
    x = matrix_NN.ldlt().solve(v_Nd);
    time_used = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_stt);
    std::cout << "Time of normal inverse is " << time_used.count() << " seconds.\n";
    std::cout << "x = " << x.transpose() << std::endl;

    return 0;
}