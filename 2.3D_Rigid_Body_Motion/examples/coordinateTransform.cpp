#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>


int main(int argc, char** argv) {
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize();
    q2.normalize();
    Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    Eigen::Vector3d p1(0.5, 0, 0.2);

    Eigen::Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    Eigen::Vector3d p2 = T2w * T1w.inverse() * p1;
    std::cout << std::endl << p2.transpose() << std::endl;
    return 0;
}