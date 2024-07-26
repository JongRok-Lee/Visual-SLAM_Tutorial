add_executable(run_kitti_stereo main.cpp)
target_link_libraries(run_kitti_stereo
    # myslam
    ${THIRD_PARTY_LIBS}
)