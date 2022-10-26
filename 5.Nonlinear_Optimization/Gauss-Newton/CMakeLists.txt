cmake_minimum_required(VERSION 3.0)
project(gaussNewton)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(gaussNewton gaussNewton.cpp)

target_link_libraries(gaussNewton
    ${OpenCV_LIBS}
)