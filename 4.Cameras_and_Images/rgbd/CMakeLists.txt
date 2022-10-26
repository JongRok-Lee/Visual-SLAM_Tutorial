cmake_minimum_required(VERSION 3.0)
project(rgb)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED PATHS "~/3rd_Party/Sophus/Sophus-22.10/install/share/sophus/cmake")
find_package(Pangolin REQUIRED PATHS "~/3rd_Party/Pangolin/Pangolin-0.6/install/lib/cmake")

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