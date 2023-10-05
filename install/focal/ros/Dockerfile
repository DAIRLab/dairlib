FROM ros:noetic-ros-base-focal
WORKDIR /dairlib
ENV DAIRLIB_DOCKER_VERSION_26=26
ENV ROS_PYTHON_VERSION=3
COPY . .
RUN apt-get update && apt-get install -y wget lsb-release pkg-config zip g++ zlib1g-dev unzip python
RUN apt-get install -y python3-rosinstall-generator python3-catkin-tools python3-vcstool python3-osrf-pycommon python3-vcstool python3-empy clang-12 clang-format-12
RUN set -eux \
  && apt-get install --no-install-recommends  locales \
  && locale-gen en_US.UTF-8
RUN if type sudo 2>/dev/null; then \ 
     echo "The sudo command already exists... Skipping."; \
    else \
     echo -e "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && yes | install/install_prereqs_focal.sh \
  && rm -rf /var/lib/apt/lists/*
