cmake_minimum_required(VERSION 3.0)
project(imageBasics)

find_package(OpenCV REQUIRED)

add_executable(imageBasics imageBasics.cpp)
add_executable(undistortImage undistortImage.cpp)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imageBasics
    ${OpenCV_LIBS}
)

target_link_libraries(undistortImage
    ${OpenCV_LIBS}
)
