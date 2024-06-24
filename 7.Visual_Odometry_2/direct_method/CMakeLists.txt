cmake_minimum_required(VERSION 3.0)
project(direct_method)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(direct_method main.cpp
    direct_method.cpp
)

target_link_libraries(direct_method
    ${OpenCV_LIBS}
    Sophus::Sophus
    ${Pangolin_LIBRARIES}
    stdc++fs
)