cmake_minimum_required(VERSION 3.0)
project(bundle_adjustment)

find_package(Ceres REQUIRED)

include_directories(
    ../
    ${CERES_INCLUDE_DIRS}
)

add_executable(bundle_adjustment
    main.cpp
    bundle_adjustment.cpp
)

target_link_libraries(bundle_adjustment
    ${CERES_LIBRARIES}
)