cmake_minimum_required(VERSION 3.0)
project(pose_estimation_2d2d)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)

target_link_libraries(pose_estimation_2d2d
    ${OpenCV_LIBS}
)