#!/bin/bash

# look in WORKSPACE file for appropriate commit number
DRAKE_COMMIT=$(grep -oP '(?<=DRAKE_COMMIT = ")(.*)(?=")' $(dirname "$0")/../WORKSPACE)

ubuntu_codename=$(lsb_release -sc)

# Add drake apt repository for lcm and libbot2
wget -O - https://drake-apt.csail.mit.edu/drake.asc | apt-key add
echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/focal focal main" > /etc/apt/sources.list.d/drake.list
apt-get update

echo "Using Drake commit '${DRAKE_COMMIT}'"
# Download and run drake install scripts
mkdir tmp
cd tmp
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/install_prereqs.sh"
mkdir source_distribution
cd source_distribution
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/install_prereqs.sh"
# skipping drake's "install_prereqs_user_environment.sh"
touch install_prereqs_user_environment.sh
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}.txt"
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}-test-only.txt"
cd ..
mkdir binary_distribution
cd binary_distribution
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/binary_distribution/install_prereqs.sh"
wget "https://raw.githubusercontent.com/yminchen/drake/${DRAKE_COMMIT}/setup/ubuntu/binary_distribution/packages-${ubuntu_codename}.txt"
cd ..
chmod +x install_prereqs.sh
./install_prereqs.sh
cd ..
rm -rf tmp/
# In addition to drake, install lcm and libbot2
apt install lcm libbot2

# Install Pinocchio
apt install -qqy lsb-release gnupg2 curl
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -
apt-get update
apt install -qqy robotpkg-py38-pinocchio  # Adapt your desired python version here

export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH