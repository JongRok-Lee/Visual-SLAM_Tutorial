cmake_minimum_required(VERSION 3.0)
project(rgb)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(joinMap joinMap.cpp)

target_link_libraries(joinMap
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Sophus::Sophus
)