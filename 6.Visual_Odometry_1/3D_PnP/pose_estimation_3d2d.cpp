#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/geometry.hpp>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

void find_feature_matches(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, std::vector<cv::DMatch> &matches);
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K);
void BAGaussianNewton(const std::vector<Eigen::Vector3d> &points3D, const std::vector<Eigen::Vector2d> &points2D, const cv::Mat &K, Sophus::SE3d &pose);

int main(int argc, char** argv) {
    if (argc != 4) {
        throw std::invalid_argument("usage: pose_estimation_3d2d img1 img2 depth1");
    }

    // Fetch RGB images
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR),
            img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
    assert(img1.data && img2.data && "Can not load images!");

    // Camera Intrinsics TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img1, img2, keypoints1, keypoints2, matches);
    std::cout << "In total, we get " << matches.size() << " set of feature points" << std::endl;

    // Fetch Depth images
    cv::Mat imgDepth1 = cv::imread(argv[3], cv::IMREAD_ANYDEPTH);
    assert(img1.data && img2.data && "Can not load images!");

    std::vector<cv::Point3f> pts3D;
    std::vector<cv::Point2f> pts2D;
    for (cv::DMatch m : matches) {
        uint16_t d = imgDepth1.ptr<uint16_t>(static_cast<int>(keypoints1[m.queryIdx].pt.y))[static_cast<int>(keypoints1[m.queryIdx].pt.x)];
        if (d == 0) {
            // Bad Depth
            continue;
        }
        float dd = d / 5000.0f;
        cv::Point2f p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
        pts3D.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        pts2D.push_back(keypoints2[m.trainIdx].pt);
    }

    cv::Mat r, t;

    // Call OpenCV's PnP, you can choose from EPnP, DLS and other methods;
    cv::solvePnP(pts3D, pts2D, K, cv::noArray(), r, t, false);
    
    // r is in the form of rotation vector, and converted to a rotation matrix by Rodrigues formula
    cv::Mat R;
    cv::Rodrigues(r, R);
    Eigen::Matrix4d pose;
    pose << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
            0, 0, 0, 1;

    std::vector<Eigen::Vector3d> ptsEigen3D;
    std::vector<Eigen::Vector2d> ptsEigen2D;
    for (int i = 0; i < pts3D.size(); ++i) {
        ptsEigen3D.push_back(Eigen::Vector3d(pts3D[i].x, pts3D[i].y, pts3D[i].z));
        ptsEigen2D.push_back(Eigen::Vector2d(pts2D[i].x, pts2D[i].y));
    }

    std::cout << "Calling bundle adjustment by gauss newton\n";
    Sophus::SE3d pose_gn;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    BAGaussianNewton(ptsEigen3D, ptsEigen2D, K, pose_gn);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Solvepnp by Gauss-Newton cost time : " << time_used.count() << " seconds." << std::endl;
    std::cout << "SolvePnP pose : \n" << pose << std::endl;
    std::cout << "Optimized pose : \n" << pose_gn.matrix() << std::endl;

    return 0;
}

void find_feature_matches(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, std::vector<cv::DMatch> &matches) {
    // ORB Extraction
    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    // Brute Fore KNN matcher
    std::vector<std::vector<cv::DMatch>> knnMatches;
    std::vector<cv::DMatch> ratioTestMatches;
    ratioTestMatches.reserve(knnMatches.size());
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2);

    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < 0.75 * knnMatches[i][1].distance) {
            ratioTestMatches.push_back(knnMatches[i][0]);
        }
    }

    std::sort(ratioTestMatches.begin(), ratioTestMatches.end());
    int threshold = ratioTestMatches.size() * 0.5;

    for (int i = 0; i < threshold; ++i) {
        matches.push_back(ratioTestMatches[i]);
    }
}

cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    double fx = K.at<double>(0, 0),
           fy = K.at<double>(1, 1),
           cx = K.at<double>(0, 2),
           cy = K.at<double>(1, 2);
    
    double u = (p.x - cx) / fx,
           v = (p.y - cy) / fy;
    
    return cv::Point2f(u, v);
}

void BAGaussianNewton(const std::vector<Eigen::Vector3d> &points3D, const std::vector<Eigen::Vector2d> &points2D, const cv::Mat &K, Sophus::SE3d &pose) {
    const int interations = 10;
    double cost = 0, lastCost = 0,
           fx = K.at<double>(0, 0),
           fy = K.at<double>(1, 1),
           cx = K.at<double>(0, 2),
           cy = K.at<double>(1, 2);

    for (int iter = 0; iter < interations; ++iter) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Vector6d b = Eigen::Vector6d::Zero();

        cost = 0;
        // Compute Cost
        for (int i = 0; i < points3D.size(); ++i) {
            // 3D Camera Coordinates Points
            Eigen::Vector3d pointCam = pose * points3D[i];
            double zInv = 1.0 / pointCam[2];
            double zInv2 = zInv * zInv;
            
            // 2D pixel Coordinates Points
            double u = fx * pointCam[0] * zInv + cx;
            double v = fy * pointCam[1] * zInv + cy;
            Eigen::Vector2d pointPixel(u, v);

            // Reprojection Error
            Eigen::Vector2d e = points2D[i] - pointPixel;
            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx*zInv, 0, fx*pointCam[0]*zInv2, fx*pointCam[0]*pointCam[1]*zInv2, -fx-fx*pointCam[0]*pointCam[0]*zInv2, fx*pointCam[1]*zInv,
                  0, -fy*zInv, fy*pointCam[1]*zInv, fy+fy*pointCam[1]*pointCam[1]*zInv2, -fy*pointCam[0]*pointCam[1]*zInv2, -fy*pointCam[0]*zInv; 

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }
        Eigen::Vector6d dx = H.ldlt().solve(b);

        if (isnan(dx[0])) {
            std::cout << "Result is NaN!\n";
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // Cost increase, Update is not good.
            std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
            break;
        }

        // Update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        std::cout << "Iteration " << iter << "\t cost = " << std::cout.precision(12) << cost << std::endl;
        if (dx.norm() < 1e-6) {
            // Convergence!
            break;
        }
    }
}
