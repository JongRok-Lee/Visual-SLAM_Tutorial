cmake_minimum_required(VERSION 3.0)
project(pose_graph)

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${g2o_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_graph pose_graph_g2o_lie_algebra.cpp)

target_link_libraries(pose_graph
    g2o::core
    g2o::stuff
    g2o::types_slam3d
    Sophus::Sophus
)