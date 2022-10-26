cmake_minimum_required(VERSION 2.8)
project(GeometyExample)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED PATHS "~/3rd_Party/Pangolin/Pangolin-0.6/install/lib/cmake")

if(${Eigen3_FOUND})
  message(STATUS "Found: Eigen3 - ${EIGEN3_INCLUDE_DIRS}")
endif(${Eigen3_FOUND})

if(${Pangolin_FOUND})
  message(STATUS "Found: Pangolin - ${Pangolin_INCLUDE_DIRS}")
endif(${Pangolin_FOUND})

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(coordinateTransform coordinateTransform.cpp)
add_executable(plotTrajectory plotTrajectory.cpp)

target_link_libraries(plotTrajectory
    ${Pangolin_LIBRARIES}
)
