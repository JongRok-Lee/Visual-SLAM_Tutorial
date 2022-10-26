cmake_minimum_required( VERSION 2.8 )
project( geometry )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 REQUIRED)

if(${Eigen3_FOUND})
  message(STATUS "Found: Eigen3 - ${EIGEN3_INCLUDE_DIRS}")

endif(${Eigen3_FOUND})

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)
add_executable(eigenGeometry eigenGeometry.cpp)
