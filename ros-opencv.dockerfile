# docker build --no-cache --progress=tty --force-rm -f ros-opencv.dockerfile -t ros-opencv:base .
# docker run -it -v /home/jiwon/workspace:/workspace -v /tmp/.X11-unix:/tmp/.X11-unix --name "ROS_OpenCV" --shm-size=14G -e DISPLAY=$DISPLAY ros-opencv:base /bin/bash

FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    lsb-release \
    curl \
    gnupg

### Setup your sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

### Set up your keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

### Installation
RUN apt update && apt install -y ros-melodic-desktop-full

### Environment setup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# source ~/.bashrc

### Dependencies for building packages
RUN apt install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential

##### Initialize rosdep
RUN rosdep init && rosdep update

######################## OpenCV ###########################
RUN apt install -y \
    build-essential \
    cmake \
    git \
    libgtk-3-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    libv4l-dev

RUN git clone https://github.com/opencv/opencv.git /workspace/opencv && \
    cd /workspace/opencv && \
    git checkout 4.5.5 && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        .. && \
    make -j$(($(nproc)-1)) && \
    make install
