ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV DEBIAN_FRONTEND noninteractive

# Core Linux tools
RUN apt-get update && apt-get install -y --no-install-recommends\
    apt-utils lsb-release sudo unzip wget less ssh vim curl\
    software-properties-common python3-dev python3-pip clang-12 \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-12 12 \
    && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-12 12 \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip --version

RUN apt-get update && apt-get install -y \
    freeglut3-dev \
    graphviz \
    libarmadillo-dev \
    libboost-iostreams-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libcereal-dev \
    libedit-dev \
    libfmt-dev \
    libncurses5-dev \
    libopenblas-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    libxmu-dev \
    libyaml-cpp-dev \
    ros-${ROS_DISTRO}-ament-cmake-nose \
    ros-${ROS_DISTRO}-eigenpy \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-yaml-cpp-vendor

# Install packages for code formatting and documentation.
RUN apt-get update && apt-get install -y doxygen clang-format \
    && python3 -m pip install black cmakelang breathing-cat

# Tools for Debugging
RUN apt-get update && apt-get install -y \
    clang-tidy \
    gdb \
    valgrind \
    tmux \
    less \
    ipython3 \
    python3-ipdb \
    python3-psutil \
    python3-venv

RUN apt-get update && apt-get install -y \
    python3-empy \
    python3-matplotlib \
    python3-opencv \
    python3-pandas \
    python3-progressbar \
    python3-scipy \
    python3-tabulate \
    python3-urwid \
    python3-yaml \
    python3-zarr

RUN python3 -m pip install line-profiler \
    numpy==1.23.3 \
    pybullet \
    gym \
    namegenerator

# Install Pylon Camera SDK
ARG pkgarch="amd64"
ARG version="5.2.0.13457"
ARG checksum=9d4f70aae93012d6ca21bb4aff706ce409da155a446e86b90d00f0dd0a26fd55
ARG pkg=pylon_${version}-deb0_${pkgarch}.deb
ARG url="https://dnb-public-downloads-misc.s3.eu-central-1.amazonaws.com/pylon/${pkg}"

RUN wget --no-check-certificate -O ${pkg} ${url} \
    # verify checksum (to make sure the package on the server wasn't modified)
    && echo "${checksum} ${pkg}" | sha256sum --check || exit 2 \
    && dpkg -i ${pkg} \
    && rm ${pkg}

RUN apt-get clean && pip cache purge

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash \
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \
    source /usr/share/colcon_cd/function/colcon_cd.sh" > /setup.bash

