cmake_minimum_required(VERSION 3.0)
project(pose_estimation_3d3d)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)

include_directories(
    ../
    ${OpenCV_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)

target_link_libraries(pose_estimation_3d3d
    ${OpenCV_LIBS}
    Sophus::Sophus
    g2o::core
    g2o::stuff
)