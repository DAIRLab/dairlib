#!/bin/bash

# look in WORKSPACE file for appropriate commit number
DRAKE_COMMIT=$(grep -oP '(?<=DRAKE_COMMIT = ")(.*)(?=")' $(dirname "$0")/../WORKSPACE)

ubuntu_codename=$(lsb_release -sc)

# Add drake apt repository for lcm and libbot2
wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/${ubuntu_codename} ${ubuntu_codename} main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
apt-get update

echo "Using Drake commit '${DRAKE_COMMIT}'"
# Download and run drake install scripts
mkdir tmp
cd tmp
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/install_prereqs.sh"
mkdir source_distribution
cd source_distribution
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/install_prereqs.sh"
# skipping drake's "install_prereqs_user_environment.sh"
touch install_prereqs_user_environment.sh
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}.txt"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}-clang.txt"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}-test-only.txt"
cd ..
mkdir binary_distribution
cd binary_distribution
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/binary_distribution/install_prereqs.sh"
wget "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/binary_distribution/packages-${ubuntu_codename}.txt"
cd ..
chmod +x install_prereqs.sh
./install_prereqs.sh
cd ..
rm -rf tmp/
# In addition to drake, install lcm and libbot2
apt install lcm libbot2