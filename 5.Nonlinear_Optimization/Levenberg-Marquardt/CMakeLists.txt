cmake_minimum_required(VERSION 3.0)
project(LevenbergMarquardt)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(LevenbergMarquardt LevenbergMarquardt.cpp)

target_link_libraries(LevenbergMarquardt
    ${OpenCV_LIBS}
)