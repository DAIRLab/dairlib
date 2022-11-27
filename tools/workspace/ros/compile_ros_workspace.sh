#!/usr/bin/env bash
# Only tested on Ubuntu 18.04
BASE_DIR="$PWD"

cd $(dirname "$BASH_SOURCE")

set -e

PACKAGES="roscpp rospy std_msgs geometry_msgs sensor_msgs tf"
# You can add any local ROS packages you need, as long as you source
# <YourCatkinWorkspace>/install/setup.bash before running this script
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

catkin build

cd $BASE_DIR