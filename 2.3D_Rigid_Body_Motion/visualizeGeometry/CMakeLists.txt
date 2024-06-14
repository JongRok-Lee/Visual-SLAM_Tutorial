cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

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

add_executable(visualizeGeometry visualizeGeometry.cpp)

target_link_libraries(visualizeGeometry
    ${Pangolin_LIBRARIES}
    )
