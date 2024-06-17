cmake_minimum_required(VERSION 3.0)
project(triangulation)

find_package(OpenCV REQUIRED)

include_directories(
    ../
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(triangulation triangulation.cpp)

target_link_libraries(triangulation
    ${OpenCV_LIBS}
)