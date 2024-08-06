cmake_minimum_required(VERSION 3.0)
project(myslam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(spdlog REQUIRED)
find_package(GTest REQUIRED)
find_package(gflags REQUIRED)
find_package(CSparse REQUIRED)
find_package(YAML-CPP REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES} GL GLU GLEW glut
    g2o::core g2o::stuff g2o::types_sba g2o::solver_csparse g2o::csparse_extension
    ${GTEST_BOTH_LIBRARIES}
    spdlog::spdlog
    ${GFLAGS_LIBRARIES}
    pthread
    ${CSPARSE_LIBRARY}
    ${YAML_CPP_LIBRARIES}
)

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
# add_subdirectory(src)
# add_subdirectory(test)
add_subdirectory(app)
