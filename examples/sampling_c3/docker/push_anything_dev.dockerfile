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

COPY snopt7.6.tar.gz /snopt7.6.tar.gz
ENV SNOPT_PATH=/snopt7.6.tar.gz

COPY gurobi10.0.3_linux64.tar.gz /tmp/gurobi.tar.gz
RUN mkdir -p /opt/gurobi && tar xzf /tmp/gurobi.tar.gz -C /opt/gurobi/ && rm /tmp/gurobi.tar.gz
ENV GUROBI_HOME=/opt/gurobi/gurobi1003/linux64
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

# install_prereqs creates ubuntu (uid 1000)
RUN usermod -l pushanything -d /home/pushanything -m ubuntu

USER pushanything
WORKDIR /home/pushanything
