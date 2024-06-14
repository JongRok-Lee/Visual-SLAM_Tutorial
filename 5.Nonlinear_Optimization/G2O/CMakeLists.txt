cmake_minimum_required(VERSION 3.0)
project(g2oCurveFitting)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

include_directories(
    ${g2o_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(g2oCurveFitting g2oCurveFitting.cpp)

target_link_libraries(g2oCurveFitting
    ${OpenCV_LIBS}
    g2o::core
    g2o::stuff
)