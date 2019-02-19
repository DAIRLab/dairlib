#!/bin/bash

DRAKE_COMMIT=$(grep -oP '(?<=DRAKE_COMMIT = ")(.*)(?=")' $(dirname "$0")/../WORKSPACE)

echo "Using Drake commit '${DRAKE_COMMIT}'"
wget --quiet -O drake_binary_brewfile \
  "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/binary_distribution/Brewfile"
wget --quiet -O drake_source_brewfile \
  "https://raw.githubusercontent.com/RobotLocomotion/drake/${DRAKE_COMMIT}/setup/mac/source_distribution/Brewfile"
# We blacklist kcov-35 because it is only available from the Drake PPA, and we don't want to add that PPA to sources.list.
sed -i 's#^kcov-35$##g' drake_source_brewfile
/usr/local/bin/brew bundle --file="drake_binary_brewfile"
/usr/local/bin/brew bundle --file="drake_source_brewfile"
rm drake_binary_brewfile
rm drake_source_brewfile