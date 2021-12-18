#!/bin/bash

echo "This install script only works for Ubuntu 20.04"

# Add drake apt repository for lcm and libbot2
wget -O - https://drake-apt.csail.mit.edu/drake.asc | apt-key add
echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/focal focal main" > /etc/apt/sources.list.d/drake.list
# Install
apt-get update
apt install lcm libbot2
