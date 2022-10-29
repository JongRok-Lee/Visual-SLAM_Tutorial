cmake_minimum_required(VERSION 3.0)
project(orb_ex)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(orb_cv orb_cv.cpp)
add_executable(orb_self orb_self.cpp)

target_link_libraries(orb_cv
    ${OpenCV_LIBS}
)

target_link_libraries(orb_self
    ${OpenCV_LIBS}
)