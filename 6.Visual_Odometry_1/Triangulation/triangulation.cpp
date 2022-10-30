#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

void triangulation(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, const std::vector<cv::DMatch> &matches, const cv::Mat &R, const cv::Mat &t, std::vector<cv::Point3d> &points);
void find_feature_matches(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, std::vector<cv::DMatch> &matches);
void pose_estimation_2d2d(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, const std::vector<cv::DMatch> &matches, const cv::Mat &K, cv::Mat &R, cv::Mat &t);
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K);
cv::Scalar get_color(float depth);

int main(int argc, char **argv) {
    if (argc != 3) {
        throw std::invalid_argument("usage: pose_estimation_2d2d img1 img2");
    }

    // Fetch images
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR),
            img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
    assert(img1.data && img2.data && "Can not load images!");

    // Camera Intrinsics TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img1, img2, keypoints1, keypoints2, matches);
    std::cout << "In total, we get " << matches.size() << " set of feature points" << std::endl;

    // Estimate the Motion between two frames
    cv::Mat R, t;
    pose_estimation_2d2d(keypoints1, keypoints2, matches, K, R, t);

    std::vector<cv::Point3d> points;
    triangulation(keypoints1, keypoints2, matches, R, t, points);

    cv::Mat imgPlot1 = img1.clone(), imgPlot2 = img2.clone();
    for (int i = 0; i < matches.size(); ++i) {
        float depth1 = points[i].z;
        std::cout << "Depth : " << depth1 << std::endl;
        cv::Point2d pt1Cam = pixel2cam(keypoints1[matches[i].queryIdx].pt, K);
        cv::circle(imgPlot1, keypoints1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        cv::Mat pt2Trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2Trans.at<double>(2, 0);
        cv::circle(imgPlot2, keypoints2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("Img 1", imgPlot1);
    cv::imshow("Img 2", imgPlot2);
    cv::waitKey(0);

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

void pose_estimation_2d2d(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, const std::vector<cv::DMatch> &matches, const cv::Mat &K, cv::Mat &R, cv::Mat &t) {
    // Convert the matching point to the form of std::vector<cv::Point2f>
    // matches = {queryIdx, trainIdx, distance}
    // keypoints.pt -> cv::Point2f
    std::vector<cv::Point2f> points1, points2;
    for (int i = 0; i < matches.size(); ++i) {
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    // Calculate Essential Matrix
    cv::Point2d principlePoint(325.1, 249.7);   // Camera Principle point, Calibrated in TUM dataset
    double focalLength = 521;                   // Camera Focal length, Calibrated in TUM dataset
    cv::Mat essentialMatrix;
    essentialMatrix = cv::findEssentialMat(points1, points2, focalLength, principlePoint, cv::RANSAC);
    std::cout << "Essential Matrix is " << std::endl << essentialMatrix << std::endl;

    // Recover Rotation and Translation from the Essential Matrix.
    cv::recoverPose(essentialMatrix, points1, points2, R, t, focalLength, principlePoint);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
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

void triangulation(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, const std::vector<cv::DMatch> &matches, const cv::Mat &R, const cv::Mat &t, std::vector<cv::Point3d> &points) {
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);

    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> pts1, pts2;
    for (auto &m : matches) {
        // Convert pixel coordinates to camera coordinates
        pts1.push_back(pixel2cam(keypoints1[m.queryIdx].pt, K));
        pts2.push_back(pixel2cam(keypoints2[m.trainIdx].pt, K));
    }

    cv::Mat pts4D;
    cv::triangulatePoints(T1, T2, pts1, pts2, pts4D);

    // Convert to Non-Homogeneous Coordinates
    for (int i = 0; i < pts4D.cols; ++i) {
        cv::Mat x = pts4D.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
        points.push_back(p);
    }

}

cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}