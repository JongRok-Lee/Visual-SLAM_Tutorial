# Visual-SLAM Tutorial
Introduction to visual-SLAM 책을 공부하며, 작성한 코드 레포지토리입니다. 아래 라이브러리 사용에 맞게 `CMakeLists.txt` 파일을 수정하고, 기존 [slambook2](https://github.com/gaoxiang12/slambook2) 코드들을 조금씩 개량해 작성하였습니다.

## 3rd 파티 라이브러리
OpenCV (4.2 with `ROS Noetic`)  
Eigen3 (3.3.7-2 with `libeigen3-dev`)  
Pangolin ([0.6](https://github.com/stevenlovegrove/Pangolin/releases/tag/v0.6))  
Sophus ([tag 22.10](https://github.com/strasdat/Sophus/releases/tag/v22.10))  
Ceres Solver ([2.1.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0))  
G2O ([20201223_git](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git))

## Build
서드 파티 라이브러리들을 local이 아닌 `cmake -DCMAKE_INSTALL_PREFIX`로 `~/3rd_Party/`내부에 빌드하였기 때문에 `CMakeLists.txt`에서도 `find_package(.. PATHS ..)`로 패키지 위치를 찾아주어야 합니다.  
예시)  
```
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED PATHS "~/3rd_Party/Sophus/Sophus-22.10/install/share/sophus/cmake")
find_package(Pangolin REQUIRED PATHS "~/3rd_Party/Pangolin/Pangolin-0.6/install/lib/cmake")
```
Prefix가 아닌 local에 빌드 시, `CMakeLists.txt`파일을 수정하여 사용바랍니다.


