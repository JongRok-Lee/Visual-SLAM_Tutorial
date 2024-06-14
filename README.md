# Visual-SLAM Tutorial
[Introduction to visual-SLAM](https://github.com/gaoxiang12/slambook-en) 책을 공부하며, 작성한 코드 레포지토리입니다. 아래 라이브러리 사용에 맞게 `CMakeLists.txt` 파일을 수정하고, 기존 [slambook2](https://github.com/gaoxiang12/slambook2) 코드들을 조금씩 개량해 작성하였습니다.

## 개발환경
Ubuntu 20.04  
OpenCV ([4.2.0](https://github.com/opencv/opencv/releases/tag/4.2.0))  
Eigen3 (3.3.7-2 with `libeigen3-dev`)  
Pangolin ([0.6](https://github.com/stevenlovegrove/Pangolin/releases/tag/v0.6))  
Sophus ([tag 22.10](https://github.com/strasdat/Sophus/releases/tag/1.22.10))  
Ceres Solver ([2.1.0](https://github.**com**/ceres-solver/ceres-solver/releases/tag/2.1.0))  
G2O ([20201223_git](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git))  
위 라이브러리 버전을 도커로 구성하였습니다.
### 도커 Environment 이미지
구동에 필요한 최소한의 라이브러리들이 설치된 이미지입니다.
``` shell
docker build -f docker/env.Dockerfile -t slam-env docker/
```
### User 권한
개인적으로 도커 컨테이너의 user를 Host와 동일하게 세팅하는 것을 선호하여 `.env`파일에 User 정보를 설정합니다. root로 실행하기 원할 경우 Skip 가능합니다.
```
# GID
echo $(id -g)
# UID
echo $(id -u)
# username
echo $(id -un)
```
``` shell
HOST_UID=1000
HOST_GID=1000
HOST_USER=jr
```
### 컨테이너 실행
``` shell
docker compose up --build -d
```
## 원본 코드와의 변경 사항
새로운 라이브러리 버전, 제게 익숙한 Cmake 컨벤션, C++언어에 맞게 수정하고 개량할 수 있는 부분들을 수정하였습니다.  
구체적인 수정사항은 `수정사항.md` 파일에 기입하였습니다.

