cmake_minimum_required(VERSION 3.0)
project(useSophus)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus
    Sophus::Sophus
)

