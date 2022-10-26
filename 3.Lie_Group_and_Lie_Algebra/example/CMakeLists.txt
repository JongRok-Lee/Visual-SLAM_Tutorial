cmake_minimum_required(VERSION 3.0)
project(sophusExample)

find_package(Pangolin REQUIRED PATHS "~/3rd_Party/Pangolin/Pangolin-0.6/install/lib/cmake")
find_package(Sophus REQUIRED PATHS "~/3rd_Party/Sophus/Sophus-22.10/install/share/sophus/cmake")
find_package(fmt REQUIRED)

include_directories(
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError
    ${Pangolin_LIBRARIES}
    fmt::fmt
    Sophus::Sophus
)
