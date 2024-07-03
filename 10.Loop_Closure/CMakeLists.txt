cmake_minimum_required(VERSION 3.0)
project(feature_training)

find_package(OpenCV REQUIRED)
find_package(DBoW3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${DBoW3_INCLUDE_DIRS}
)

add_executable(feature_training feature_training.cpp)
add_executable(loop_closure loop_closure.cpp)

target_link_libraries(feature_training
    ${OpenCV_LIBS}
    ${DBoW3_LIBS}
)

target_link_libraries(loop_closure
    ${OpenCV_LIBS}
    ${DBoW3_LIBS}
)