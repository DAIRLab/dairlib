# Build from examples/sampling_c3/docker (snopt/gurobi tarballs must be in this directory):
#   docker build -f push_anything_dev.dockerfile -t push-anything-env .
FROM ubuntu:noble
ARG DRAKE_VERSION=1.51.1
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    wget lsb-release pkg-config zip g++ zlib1g-dev unzip ca-certificates gnupg git \
    libopenblas-dev openjdk-17-jdk \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q https://github.com/RobotLocomotion/drake/archive/v${DRAKE_VERSION}.tar.gz \
    && tar -xzf v${DRAKE_VERSION}.tar.gz drake-${DRAKE_VERSION}/setup/ \
    && ./drake-${DRAKE_VERSION}/setup/install_prereqs --developer -y --with-bazel --with-clang \
    && rm -rf v${DRAKE_VERSION}.tar.gz drake-${DRAKE_VERSION}/

# Procman (libbot2 not in Drake apt for noble; build procman only from source).
RUN apt-get update && apt-get install -y \
    cmake build-essential liblcm-dev python3-lcm python3-pip python3-setuptools \
    libglu1-mesa-dev libgtk-3-dev \
    && git clone --depth 1 --branch drake https://github.com/RobotLocomotion/libbot2.git /tmp/libbot2 \
    && cmake -S /tmp/libbot2 -B /tmp/libbot2-build -DCMAKE_INSTALL_PREFIX=/opt/libbot2 \
    && cmake --build /tmp/libbot2-build -j"$(nproc)" --target bot2-procman \
    && cmake --install /tmp/libbot2-build \
    && rm -rf /tmp/libbot2 /tmp/libbot2-build /var/lib/apt/lists/*
ENV PATH="/opt/libbot2/bin:${PATH}"

COPY snopt7.6.tar.gz /snopt7.6.tar.gz
ENV SNOPT_PATH=/snopt7.6.tar.gz

COPY gurobi10.0.3_linux64.tar.gz /tmp/gurobi.tar.gz
RUN mkdir -p /opt/gurobi && tar xzf /tmp/gurobi.tar.gz -C /opt/gurobi/ && rm /tmp/gurobi.tar.gz
ENV GUROBI_HOME=/opt/gurobi/gurobi1003/linux64
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

# Same packages as examples/sampling_c3/sampling_generation/python_requirements.txt
RUN pip3 install --break-system-packages \
    trimesh ruamel.yaml lxml fast_simplification pyglet vhacdx

# install_prereqs creates ubuntu (uid 1000)
RUN usermod -l pushanything -d /home/pushanything -m ubuntu

USER pushanything
WORKDIR /home/pushanything
