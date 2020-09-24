#!/bin/bash

# look in WORKSPACE file for appropriate commit number
DRAKE_COMMIT=$(grep -oP '(?<=DRAKE_COMMIT = ")(.*)(?=")' $(dirname "$0")/../WORKSPACE)

ubuntu_codename=$(lsb_release -sc)

echo "Using Drake commit '${DRAKE_COMMIT}'"
# Download and run drake install scripts
mkdir tmp
cd tmp
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/install_prereqs.sh"
mkdir source_distribution
cd source_distribution
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/source_distribution/install_prereqs.sh"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/source_distribution/install_prereqs_user_environment.sh"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/source_distribution/Brewfile"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/source_distribution/requirements.txt"
cd ..
mkdir binary_distribution
cd binary_distribution
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/binary_distribution/install_prereqs.sh"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/binary_distribution/Brewfile"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/binary_distribution/requirements.txt"
cd ..
chmod +x install_prereqs.sh
./install_prereqs.sh
cd ..
rm -rf tmp/
# In addition to drake, install lcm and libbot2
# TODO: the line below is not tested, need a mac user to see if it will work!
brew install lcm libbot2