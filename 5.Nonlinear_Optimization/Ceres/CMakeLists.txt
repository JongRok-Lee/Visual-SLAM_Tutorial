cmake_minimum_required(VERSION 3.0)
project(ceresCurveFitting)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED PATHS "~/3rd_Party/Ceres/ceres-solver-2.1.0/install/lib/cmake")

include_directories(
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(ceresCurveFitting ceresCurveFitting.cpp)

target_link_libraries(ceresCurveFitting
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)