# dairlib

## Complete Build Instructions
### Build Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details)

Install Drake from source into `"my-workspace"/drake` http://drake.mit.edu/from_source.html. You do not need to build it, but prerequisites should also be installed.

### Build dairlib
1. Clone `dairlib` into the same root directory, "my-workspace/dairlib"

2. Build what you want via Bazel. From `dairlib`, `bazel build ...` will build the entire project. Drake will be built as an external dependency.

### (Optional Dependencies)
1. Install a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2'. The prerequisites installation from Drake should add the proper repo for these.
2. Install Director
  1. Install dependencies `sudo apt-get install build-essential cmake libglib2.0-dev libqt4-dev pyqt4-dev-tools libx11-dev libxext-dev libxt-dev python-dev python-lxml python-numpy python-scipy python-yaml`
  2. Install a binary copy of Drake into `/opt/drake/` (not ideal--will work on removing this dependency). 
  ```
  wget https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
  sudo tar -xzf drake-latest-xenial.tar.gz -C /opt/
  rm drake-latest-xenial.tar.gz
  ```
  3. Clone DAIRLab's fork of `director` into your workspace directory. `git clone https://github.com/DAIRLab/director.git`
  4. Build director
  ```
  export :/opt/drake/lib/cmake/drake
  mkdir build
  cd build
  cmake ../distro/superbuild/
  make
  ```

## Included Modules
A list of included modules

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction.

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf