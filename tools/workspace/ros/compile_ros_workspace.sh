#!/usr/bin/env bash

# In addition to your base ROS install,
# you must sudo apt-get install python3-vcstool

# Tested on ubuntu 20.04 and 18.04
BASE_DIR="$PWD"

cd $(dirname "$BASH_SOURCE")

set -e

PACKAGES="roscpp rospy std_msgs geometry_msgs geometry2 sensor_msgs tf grid_map_msgs rosbag"
# You can add any published ros packages you need to this list.
# Local ROS packages should be their own bazel local_repository

rm -rf bundle_ws
mkdir bundle_ws
pushd bundle_ws
mkdir src

DISTRO=$(lsb_release -c -s)

if [ $DISTRO == "bionic" ]
then
  rosinstall_generator \
    --deps \
    --tar \
    --flat \
    $PACKAGES > ws.rosinstall
  wstool init -j1 src ws.rosinstall
elif [ $DISTRO == "focal" ]
then
  rosinstall_generator \
    --rosdistro noetic \
    --deps \
    --tar \
    --flat \
    $PACKAGES > ws.rosinstall
  vcs import src < ws.rosinstall
else
  echo "${DISTRO} not supported for ROS with dairlib!"
  exit 1
fi

catkin config \
    --install \
    --source-space src \
    --build-space build \
    --devel-space devel \
    --log-space log \
    --install-space install \
    --isolate-devel \
    --no-extend

catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

cd $BASE_DIR