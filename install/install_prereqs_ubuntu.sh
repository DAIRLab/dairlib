#!/bin/bash

DRAKE_COMMIT=$(grep -oP '(?<=DRAKE_COMMIT = ")(.*)(?=")' $(dirname "$0")/../WORKSPACE)

ubuntu_codename=$(lsb_release -sc)

echo "Using Drake commit '${DRAKE_COMMIT}'"
wget --quiet -O drake_binary_packages.txt \
  "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/binary_distribution/packages-${ubuntu_codename}.txt"
wget --quiet -O drake_source_packages.txt \
  "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/ubuntu/source_distribution/packages-${ubuntu_codename}.txt"
# We blacklist kcov-35 because it is only available from the Drake PPA, and we don't want to add that PPA to sources.list.
sed -i 's#^kcov-35$##g' drake_source_packages.txt
sudo apt-get -q install --no-install-recommends $(cat drake_binary_packages.txt)
sudo apt-get -q install --no-install-recommends $(cat drake_source_packages.txt)
rm drake_binary_packages.txt
rm drake_source_packages.txt