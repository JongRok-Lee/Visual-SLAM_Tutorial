cmake_minimum_required(VERSION 3.0)
project(pose_estimation_3d2d)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED PATHS "~/3rd_Party/Sophus/Sophus-22.10/install/share/sophus/cmake")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)

target_link_libraries(pose_estimation_3d2d
    ${OpenCV_LIBS}
    Sophus::Sophus
)