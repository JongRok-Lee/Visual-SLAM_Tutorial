#include <iostream>
#include <stdexcept>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv) {
    if (argc != 3) {
        throw std::invalid_argument("usage: feature_extraction img1 img2");
    }

    // Read images
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_ANYCOLOR),
            img2 = cv::imread(argv[2], cv::IMREAD_ANYCOLOR);
    assert(img1.data != nullptr && img2.data != nullptr);

    // Initialization
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptor1, descriptor2;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

    // Detect Oriented FAST & Compute Rotated BRIEF
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector-> detectAndCompute(img1, cv::noArray(), keypoint1, descriptor1);
    detector-> detectAndCompute(img2, cv::noArray(), keypoint2, descriptor2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Extract ORB cost = " << time_used.count() << "seconds.\n";

    cv::Mat outImg1, outImg2, outImg;
    cv::drawKeypoints(img1, keypoint1, outImg1);
    cv::drawKeypoints(img2, keypoint2, outImg2);
    cv::hconcat(outImg1, outImg2, outImg);
    cv::imshow("ORB features", outImg);

    // Use Hamming distance to match the features with match
    std::vector<cv::DMatch> Matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(descriptor1, descriptor2, Matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Match ORB cost = " << time_used.count() << " seconds.\n";


    // Use Hamming distance to match the features with KnnMatch
    std::vector<std::vector<cv::DMatch>> knnMatches;
    t1 = std::chrono::steady_clock::now();
    matcher->knnMatch(descriptor1, descriptor2, knnMatches, 2);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "KnnMatch ORB cost = " << time_used.count() << " seconds.\n";

    // min and max distance
    auto minMax = std::minmax_element(Matches.begin(), Matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) {return m1.distance < m2.distance;});
    float minDist = minMax.first->distance,
          maxDist = minMax.second->distance;
    std::cout << "Max dist : " << maxDist << std::endl;
    std::cout << "Min dist : " << minDist << std::endl;


    // Remove the bad matching with Lowe ratio test
    std::vector<cv::DMatch> ratioTestMatches;
    ratioTestMatches.reserve(knnMatches.size());

    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < 0.75 * knnMatches[i][1].distance) {
            ratioTestMatches.push_back(knnMatches[i][0]);
        }
    }

    // Remove the bad matching with sort
    std::sort(ratioTestMatches.begin(), ratioTestMatches.end());
    int threshold = ratioTestMatches.size() * 0.3;
    std::vector<cv::DMatch> goodMatches(ratioTestMatches.begin(), ratioTestMatches.begin() + threshold);
    

    // Draw the match results
    cv::Mat imgMatch, imgGoodMatch;
    cv::drawMatches(img1, keypoint1, img2, keypoint2, Matches, imgMatch);
    cv::drawMatches(img1, keypoint1, img2, keypoint2, goodMatches, imgGoodMatch);
    cv::imshow("All matches", imgMatch);
    cv::imshow("Good matches", imgGoodMatch);
    cv::waitKey(0);

    return 0;
}