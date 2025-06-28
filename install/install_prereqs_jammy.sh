  #!/bin/bash

# look in WORKSPACE file for appropriate commit number
DRAKE_VERSION=$(grep -oP 'DRAKE_VERSION = "v?\K[^"]*' $(dirname "$0")/../MODULE.bazel)
echo ${DRAKE_VERSION}

ubuntu_codename=$(lsb_release -sc)

# Add drake apt repository for lcm and libbot2
wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/${ubuntu_codename} ${ubuntu_codename} main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null


wget https://github.com/RobotLocomotion/drake/archive/v${DRAKE_VERSION}.tar.gz
tar -xzf v${DRAKE_VERSION}.tar.gz drake-$DRAKE_VERSION/setup/
./drake-${DRAKE_VERSION}/setup/install_prereqs --developer -y --with-bazel
rm -rf ./drake-${DRAKE_VERSION}/
rm v${DRAKE_VERSION}.tar.gz

# In addition to drake, install lcm and libbot2
sudo apt install lcm libbot2