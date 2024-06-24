cmake_minimum_required(VERSION 3.0)
project(optical_flow)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(optical_flow main.cpp
    optical_flow.cpp
)

target_link_libraries(optical_flow
    ${OpenCV_LIBS}
)