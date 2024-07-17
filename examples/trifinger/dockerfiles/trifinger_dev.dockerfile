ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive

###### Core Linux tools #####
RUN apt-get update && apt-get install -y --no-install-recommends\
    apt-utils net-tools lsb-release sudo unzip wget less ssh vim curl\
    software-properties-common python3-dev python3-pip clang-12 rsync \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-12 12 \
    && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-12 12 \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip --version

# Install Bazel
RUN apt-get install apt-transport-https curl gnupg -y \
  && curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg \
  && mv bazel-archive-keyring.gpg /usr/share/keyrings \
  && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list \
  && apt-get update && apt-get install bazel=7.2.1 -y \

RUN apt-get update && apt-get install -y \
    bazel \
    freeglut3-dev \
    graphviz \
    libgflags-dev \
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
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-gi \
    gobject-introspection \
    r1.2-gtk-3.0

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
    pybullet \
    gym \
    namegenerator

# Install dependencies for pose estimation.
RUN python3 -m pip install xgboost \
    scikit-learn \
    opencv-python \
    opencv-contrib-python

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

# Install Clion
RUN wget https://download.jetbrains.com/cpp/CLion-2024.1.tar.gz
RUN tar xvzf CLion-2024.1.tar.gz -C /opt/
RUN rm CLion-2024.1.tar.gz


# Install dependencies for drake.
COPY install_prereqs_jammy.sh /tmp/install_prereqs_jammy.sh
RUN chmod +x /tmp/install_prereqs_jammy.sh \
    && yes | /tmp/install_prereqs_jammy.sh \
RUN ldconfig /usr/local/lib

# Poetry is for python package management.
RUN curl -sSL https://install.python-poetry.org | python3 -

# User and permissions
ARG user=trifinger
ARG group=trifinger
ARG uid=1000
ARG gid=1000
ARG home=/home/${user}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${gid} ${group} \
    && useradd -d ${home} -u ${uid} -g ${gid} -m -s /bin/bash ${user} \
    && echo "${user} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${user}
USER ${user}
RUN sudo usermod -a -G video ${user}
WORKDIR ${home}

# Bash script to build ROS workspace using colcon.
COPY ros_entrypoint.sh /usr/local/bin/ros_build
RUN echo "export PATH=$PATH:/opt/libbot2/0.0.1.20221116/bin/" >> ${home}/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ${home}/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${home}/.bashrc
RUN echo "export XDG_CACHE_HOME=${home}/git/.cache" >> ${home}/.bashrc

# upgrade bazel
RUN sudo apt install bazel -y