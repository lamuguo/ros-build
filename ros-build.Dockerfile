FROM ubuntu:20.04

RUN apt update && apt install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Fix locale issue for tzdata installation (apt install tzdata)
RUN apt update && apt install -y locales && locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN DEBIAN_FRONTEND=noninteractive TZ=America/Los_Angeles apt install -y \
  curl gnupg2 lsb-release \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  python3-rosinstall-generator \
  wget \
  python3-catkin

RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# install Fast-RTPS & Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev

RUN rosdep init && rosdep update

WORKDIR /root/ros_catkin_ws

RUN rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall && \
    mkdir ./src && \
    vcs import --input noetic-desktop.rosinstall ./src

RUN rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

RUN cd /root && \
    wget https://github.com/ros/catkin/archive/refs/tags/0.8.10.tar.gz && \
    tar zxvf 0.8.10.tar.gz && cd catkin-0.8.10 && \
    mkdir build && cd build && cmake .. && make install

RUN catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
