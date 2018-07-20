# dairlib

## Complete Build Instructions
### Build Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details)

Install Drake from source into `"my-workspace"/drake` http://drake.mit.edu/from_source.html. You do not need to build it, but prerequisites should also be installed. You will need `git` to start.

### Build dairlib
1. Clone `dairlib` into the same root directory, "my-workspace/dairlib"
```
git clone https://github.com/DAIRLab/dairlib.git
```

2. Build what you want via Bazel. From `dairlib`, `bazel build ...` will build the entire project. Drake will be built as an external dependency.

### (Optional Dependencies)
These dependencies are necessary for some advanced visualization and process management. Many examples will work without a full installation of Director or libbot, but (for lab members), these are ultimately recommended. The build process below is not ideal (particularly the extra installation of binary Drake), but we will try to improve the process.
1. Install a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2'. The prerequisites installation from Drake should add the proper repo for these.
2. Install Director
  1. Install dependencies
  ```
  sudo apt-get install build-essential cmake libglib2.0-dev libqt4-dev \
        pyqt4-dev-tools libx11-dev libxext-dev libxt-dev python-dev python-lxml \
        python-numpy python-scipy python-yaml libqwt-dev
  ```
  2. Install a binary copy of Drake into `/opt/drake/` (not ideal--will work on removing this dependency). Replace `xenial` with `mac` in the following commands if building on macOS.
  ```
  wget https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
  sudo tar -xzf drake-latest-xenial.tar.gz -C /opt/
  rm drake-latest-xenial.tar.gz
  ```
  3. Clone DAIRLab's fork of `director` into your workspace directory.
  ```
  git clone https://github.com/DAIRLab/director.git
  ```
  4. Build director
  ```
  export CMAKE_PREFIX_PATH=/opt/drake/lib/cmake
  mkdir build
  cd build
  cmake ../distro/superbuild/
  make
  ```

### Notes for macOS
1. Bazel will expect there to be a python executable in `/usr/local/opt/python@2/libexec/bin/`, though this folder might not exist or be empty depending on your python configuration before building. If that is the case, construct a symlink to an appropriate executable for python 2.7, which may look similar to the following:
```
ln -s /usr/local/opt/python@2/bin/python /usr/local/opt/python@2/libexec/bin/python
```

2. Drake Director will require qt 4.8.x or 4.7.x, which are no longer available through Homebrew by standard. A workaround can be found at https://github.com/cartr/homebrew-qt4.

3. Drake Director may require qwt, which is available in the Homebrew package `qwt`.

4. Be sure to have Xcode 9.0 or later installed with Command Line Tools. If you receive a `clang: error: cannot specify -o when generating multiple output files` message during the build process, re-run `install_prereqs.sh`, and be sure that it runs fully before termination, as this will reconfigure Xcode to work with Drake.

## Included Modules
A list of included modules

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf
