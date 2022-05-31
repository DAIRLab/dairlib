#!/usr/bin/env bash
# Only tested on Ubuntu 18.04
BASE_DIR="$PWD"

cd $(dirname "$BASH_SOURCE")

set -e

PACKAGES="roscpp rospy geometry_msgs"

rm -rf bundle_ws
mkdir bundle_ws
pushd bundle_ws
mkdir src

rosinstall_generator \
    --rosdistro noetic \
    --deps \
    --tar \
    --flat \
    $PACKAGES > ws.rosinstall
vcs import src < ws.rosinstall

catkin config \
    --install \
    --source-space src \
    --build-space build \
    --devel-space devel \
    --log-space log \
    --install-space install \
    --isolate-devel \
    --no-extend

catkin build

cd $BASE_DIR