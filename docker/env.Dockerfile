FROM nvcr.io/nvidia/tensorrt:23.04-py3

# Set environment variables
ENV NVENCODE_CFLAGS "-I/usr/local/cuda/include"
ENV CV_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV NVIDIA_VISIBLE_DEVICES="all"
ENV NVIDIA_DRIVER_CAPABILITIES="all"

# Get all dependencies
RUN apt-get update && apt-get install -y \
    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim \
    python3-pip python3-tornado python3-dev python3-numpy python3-virtualenv \
    libsuitesparse-dev libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev \
    freeglut3-dev libglu1-mesa-dev mesa-common-dev libglew-dev libfmt-dev \
    libgtest-dev libopenblas-dev libgmp-dev libmpfr-dev libpng-dev libtiff-dev libdc1394-22-dev xfce4-terminal bash-completion sudo &&\
    rm -rf /var/lib/apt/lists/*

# OpenCV
WORKDIR /opencv
RUN git clone https://github.com/opencv/opencv.git -b $CV_VERSION
WORKDIR /opencv/opencv/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /opencv
WORKDIR /
ENV OpenCV_DIR=/usr/share/OpenCV

# Pangolin
WORKDIR /pangolin
RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip && unzip v0.6.zip && rm v0.6.zip
WORKDIR /pangolin/Pangolin-0.6/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /pangolin

# Sophus
WORKDIR /sophus
RUN wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.zip && unzip 1.22.10.zip && rm 1.22.10.zip
WORKDIR /sophus/Sophus-1.22.10/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /sophus

# ceres
WORKDIR /ceres
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip && unzip 2.1.0.zip && rm 2.1.0.zip
WORKDIR /ceres/ceres-solver-2.1.0/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /ceres

# g2o
WORKDIR /g2o
RUN wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20201223_git.zip && unzip 20201223_git.zip && rm 20201223_git.zip
WORKDIR /g2o/g2o-20201223_git/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /g20

# DBow3
WORKDIR /
RUN git clone https://github.com/rmsalinas/DBow3.git
WORKDIR /DBow3/build
RUN cmake .. &&\
make -j12 &&\
make install &&\
ldconfig &&\
rm -rf /DBow3
