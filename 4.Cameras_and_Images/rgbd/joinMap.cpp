#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include "pangolin/pangolin.h"
#include "sophus/se3.hpp"


typedef std::vector<Sophus::SE3d> TrajectoryType;
namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

void showPointCloud(const std::vector<Eigen::Vector6d> &pointcloud);

int main(int argc, char **argv) {
    std::vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType poses;
    poses.reserve(7);

    std::ifstream fin("../pose.txt");
    if (!fin) {
        std::cerr << "Please run the program in the directory that has pose.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 5; ++i) {
        boost::format fmt("../%s/%d.%s");       // The imgae format (../color/1.png)
        colorImgs.push_back(cv::imread((fmt % "color" % (i+1) % "png").str(), cv::IMREAD_ANYCOLOR));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i+1) % "pgm").str(), cv::IMREAD_ANYDEPTH));

        double data[7] = {0};
        for (auto &d : data) {
            fin >> d;
        }
        
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                          Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    // Compute the point cloud using camera Intrinsics
    double cx = 325.5,
           cy = 253.5,
           fx = 518.0,
           fy = 519.0,
           depthScale = 1000.0;
    std::vector<Eigen::Vector6d> pointcloud;
    pointcloud.reserve(colorImgs[0].rows * colorImgs[0].cols * 7);

    for (size_t i = 0; i < 5; ++i) {
        std::cout << "Converting RGBD images " << i+1 << std::endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];

        for(size_t v = 0; v < color.rows; ++v) {
            for(size_t u = 0; u < color.cols; ++u) {

                uint32_t d = depth.ptr<uint16_t>(v)[u];     // depth value is uint16.
                if (d == 0) {
                    continue;
                }
                Eigen::Vector3d point;
                point(2, 0) = static_cast<double>(d) / depthScale;
                point(0, 0) = (u - cx) * point(2, 0) / fx;
                point(1, 0) = (v - cy) * point(2, 0) / fy;
                Eigen::Vector3d pointWorld = T * point;

                Eigen::Vector6d p;
                p.head<3>() = pointWorld;
                cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
                p[5] = bgr[0];      // Blue
                p[4] = bgr[1];      // Green
                p[3] = bgr[2];      // Red
                pointcloud.push_back(p);
            }
        }
    }

    std::cout << "global point cloud has " << pointcloud.size() << " points." << std::endl;
    showPointCloud(pointcloud);

    return 0;    
}

void showPointCloud(const std::vector<Eigen::Vector6d> &pointcloud) {

    if (pointcloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));   // sleep 5 ms
    }
    return;
}
