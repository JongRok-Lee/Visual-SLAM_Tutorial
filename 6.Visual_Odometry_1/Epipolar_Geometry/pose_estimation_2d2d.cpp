#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

void find_feature_matches(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, std::vector<cv::DMatch> &matches);
void pose_estimation_2d2d(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, const std::vector<cv::DMatch> &matches, const cv::Mat &K, cv::Mat &R, cv::Mat &t);
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

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

    // Check E=t^R*scale
    double t1 = t.at<double>(0, 0),
           t2 = t.at<double>(1, 0),
           t3 = t.at<double>(2, 0);
    cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t3, t2,
                                             t3, 0, -t1,
                                             -t2, t1, 0);
    std::cout << "t^R=" << std::endl << t_x * R << std::endl;

    // Check Epipolar Constraint
    for (auto m : matches) {
        cv::Point2d pt1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
        cv::Point2d pt2 = pixel2cam(keypoints2[m.trainIdx].pt, K);

        cv::Mat y1 = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);
        cv::Mat y2 = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);
        cv::Mat d = y2.t() * t_x * R * y1;
        std::cout << "Epipolar constraint = " << d << std::endl;
    }

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

    // Calculate Fundamental Matrix
    cv::Mat fundamentalMatrix;
    fundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    std::cout << "Fundamental Matrix is " << std::endl << fundamentalMatrix << std::endl;

    // Calculate Essential Matrix
    cv::Point2d principlePoint(325.1, 249.7);   // Camera Principle point, Calibrated in TUM dataset
    double focalLength = 521;                   // Camera Focal length, Calibrated in TUM dataset
    cv::Mat essentialMatrix;
    essentialMatrix = cv::findEssentialMat(points1, points2, K, cv::RANSAC);
    std::cout << "Essential Matrix is " << std::endl << essentialMatrix << std::endl;

    // Calculate Homography
    // But the scene is not planer, and calculation the homography matrix here is of little significance.
    cv::Mat homographyMatrix;
    homographyMatrix = cv::findHomography(points1, points2, cv::RANSAC, 3.0);
    std::cout << "Homography Matrix is " << std::endl << homographyMatrix << std::endl;

    // Recover Rotation and Translation from the Essential Matrix.
    cv::recoverPose(essentialMatrix, points1, points2, K, R, t);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    double fx = K.at<double>(0, 0),
           fy = K.at<double>(1, 1),
           cx = K.at<double>(0, 2),
           cy = K.at<double>(1, 2);
    
    double u = (p.x - cx) / fx,
           v = (p.y - cy) / fy;
    
    return cv::Point2d(u, v);
}