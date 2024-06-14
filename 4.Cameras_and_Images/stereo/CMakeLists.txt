cmake_minimum_required(VERSION 3.0)
project(stereo)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(stereoVision stereoVision.cpp)

target_link_libraries(stereoVision
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)