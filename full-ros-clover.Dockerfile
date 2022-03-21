# Use the version under development can get latest python packages
# and easy to pass rosdep check.
FROM ubuntu:20.04

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

RUN apt update && apt install -y sudo wget lsb-release add-apt-key

# Add gazebo11 repo
RUN wget -O- http://packages.osrfoundation.org/gazebo.key | gpg --dearmor | sudo tee /usr/share/keyrings/gazebo-archive-keyring.gpg && \
echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add no-password sudo user docker
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER docker

RUN sudo apt update && sudo apt install -y locales
RUN sudo locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN sudo DEBIAN_FRONTEND=noninteractive TZ=America/Los_Angeles apt install -y \
  curl gnupg2 lsb-release \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python-is-python3 \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-setuptools \
  python3-rosinstall-generator \
  wget \
  python3-catkin

RUN sudo python3 -m pip install -U \
  vcstool \
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

RUN sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev

ENV ROS_VERSION         1
ENV ROS_PYTHON_VERSION  3
ENV ROS_DISTRO          noetic

ADD missed_debs /tmp/missed_debs
RUN sudo dpkg --force-all -i /tmp/missed_debs/*.deb

RUN sudo rosdep init || echo "sources list file already exists"
RUN rosdep update && mkdir /home/docker/catkin_ws
WORKDIR /home/docker/catkin_ws
# TODO(lamuguo): remove mavlink and enable mavlink in checking out PX4.
RUN rosinstall_generator desktop_full mavros mavlink --rosdistro noetic --deps --tar > clover.rosinstall && \
    mkdir ./src && \
    vcs import --input clover.rosinstall ./src

# To ignore interactive for keyboard-configuration
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install lightdm -y
RUN rosdep install --from-paths ./src --ignore-packages-from-source -y

RUN cd /home/docker && \
    wget https://github.com/ros/catkin/archive/refs/tags/0.8.10.tar.gz && \
    tar zxvf 0.8.10.tar.gz && cd catkin-0.8.10 && \
    mkdir build && cd build && cmake .. && sudo make install

RUN catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DROS_PYTHON_VERSION=3

RUN cd src && \
    git clone --depth 1 https://github.com/CopterExpress/clover && \
    git clone --depth 1 https://github.com/CopterExpress/ros_led && \
    git clone --depth 1 https://github.com/ethz-asl/mav_comm

RUN rosinstall_generator cv_camera mavros_extras rosbridge_server web_video_server tf2_web_republisher ros_pytest --rosdistro noetic --deps --tar > addition.rosinstall && \
    vcs import --input addition.rosinstall ./src/
RUN rosdep update && rosdep install --rosdistro neotic --from-paths src --ignore-src -y 

RUN git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git /home/docker/PX4-Autopilot
RUN ln -s /home/docker/PX4-Autopilot /home/docker/catkin_ws/src/ && \
    ln -s /home/docker/PX4-Autopilot/Tools/sitl_gazebo /home/docker/catkin_ws/src/ && \
    cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup && \
    ./ubuntu.sh && \
    ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
# TODO(lamuguo): enable mavlink below.
# RUN ln -s /home/docker/PX4-Autopilot/mavlink /home/docker/catkin_ws/src/

RUN sudo /home/docker/catkin_ws/install_isolated/lib/mavros/install_geographiclib_datasets.sh
RUN pip install -r ~/catkin_ws/src/clover/clover/requirements.txt && pip install toml

# RUN make px4_sitl gazebo_solo
# RUN catkin_make_isolated

# Almost done. Gotten error in compiling mavlink_sitl_gazebo module.

