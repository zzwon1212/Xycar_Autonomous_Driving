FROM nvcr.io/nvidia/tensorrt:20.11-py3

ENV DEBIAN_FRONTEND=noninteractive



######################## ROS ###########################
RUN apt-get update && \
    apt-get install -y \
    lsb-release \
    curl

### Setup your sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
           /etc/apt/sources.list.d/ros-latest.list'

### Set up your keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    apt-key add -

### Installation
RUN apt-get update && \
    apt-get install -y \
    ros-melodic-ros-base \
    ros-melodic-image-transport \
    ros-melodic-camera-info-manager

### Environment setup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    /bin/bash -c "source ~/.bashrc"

### Dependencies for building packages
RUN apt-get install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential

### Initialize rosdep
RUN rosdep init && \
    rosdep update



######################## OpenCV ###########################
RUN apt-get install -y \
    python3-dev \
    python3-numpy \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtk-3-dev \
    libjpeg-dev

RUN mkdir -p /home/opencv && \
    curl -sSL https://github.com/opencv/opencv/archive/refs/tags/4.5.5.tar.gz | \
    tar -xz --strip-components=1 -C /home/opencv && \
    cd /home/opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          .. && \
    make -j$(($(nproc))) && \
    make install && \
    cp lib/python3/cv2.cpython-36m-x86_64-linux-gnu.so /usr/lib/python3/dist-packages/



# ######################## Python3 ###########################
RUN sed -i '1s/python/python3/' $(which pip3)

RUN pip3 install \
    rospkg \
    Pillow \
    pyyaml \
    torch==1.8.0 \
    torchvision==0.9.0 \
    onnx==1.9.0 \
    gdown
