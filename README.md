# dairlib
Warning! This is very much "development-level" code and is provided as-is. APIs are likely to be unstable and, while we hope for the documentation to be thorough and accurate, we make no guarantees.

## Current Continuous Integration Status
* `master` branch build and unit tests (Ubuntu 18.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* `master` branch build and unit tests (Ubuntu 20.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_focal&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* `master` branch build and unit tests (Ubuntu 20.04 with ROS): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_with_ros&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* Experimental build against Drake's `master` branch: [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=drake_master_build&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
## Complete Build Instructions

### Download dairlib
1. Clone `dairlib` into the your workspace, e.g. "my-workspace/dairlib".
```
git clone https://github.com/DAIRLab/dairlib.git
```

2. Download and setup SNOPT

dairlib, by default, assumes that users have access to SNOPT(https://web.stanford.edu/group/SOL/snopt.htm), though it is not required. **If you do not have SNOPT**, you will need to edit `.bazelrc` and change `build --define=WITH_SNOPT=ON` to `build --define=WITH_SNOPT=OFF`

For users at Penn, download SNOPT (https://www.seas.upenn.edu/~posa/snopt/snopt7.6.tar.gz) and add the following line to your `~/.bashrc`
```
export SNOPT_PATH=<the directory you downloaded to>/snopt7.6.tar.gz
```

There is no need to extract the tar.

### Build Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details). There are two ways to use Drake within dairlib:

#### Option 1: use pegged revision (Note - These steps may need repeated if switching to a branch with a different pegged revision of drake).
The only specific action needed here is to install all of Drake's prerequisites. There are two choices for completing this step:

a) In `dairlib/install`, run the appropriate `install_prereqs_xxx.sh`. This is untested on mac, and has not been tested to get every dependency for a fresh install.

b) Download a source copy of drake, and install pre-requisites as described here: http://drake.mit.edu/from_source.html. Drake dependencies can change without notice. For full compatiability, you may need to checkout the drake commit which is pegged in WORKSPACE to install the correct dependencies. There is no need to build Drake itself. Proceed only until you have run the Drake setup script. 

bazel will automatically download the pegged revision, specified in the WORKSPACE file. dairlib developers hope to keep this pegged revision current, and ensure that the pegged version will always work with a specific version of dairlib.

This option is recommended for users who are not currently editing any source code in Drake itself.

#### Option 2: source install of Drake
Complete both steps (a) and (b) above. By running the drake install script after the dairlib install script, you are capturing any dependency changes between the pegged revision and the current Drake master, while still getting any aditional dairlib dependencies we may add. There is no need to build Drake. Next, to tell dairlib to use your local install, set the environment variable `DAIRLIB_LOCAL_DRAKE_PATH`, e.g.
```
export DAIRLIB_LOCAL_DRAKE_PATH=/home/user/my-workspace/drake
```

### IDE setup
JetBrains IDEs have worked well for us and are available for free to students. For C++ development using the CLion Bazel plugin, see https://drake.mit.edu/clion.html and replace `drake` with `dairlib` in the "Setting up Drake in CLion" section. 

### Other dependencies
These dependencies are necessary for some advanced visualization and process management. Many examples will work without a full installation of Director or libbot, but (for lab members), these are ultimately recommended. 

#### LCM and libbot
Install a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2`. The prerequisites installation (option 1.a) should add the proper apt repo for these.

#### ROS
To integrate with ROS (tested on ROS Noetic with 20.04), the following steps are required.
1. Install ROS http://wiki.ros.org/ROS/Installation
2. Do not forget to setup your environment. For instance, add these lines to `~/.bashrc`
```
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash 
```
and then run `source ~/.bashrc`

3. Install additional dependencies
```
sudo apt install python3-rosinstall-generator python-catkin-tools python3-vcstool
```
4. Build the ROS workspace using catkin. From `dairlib/`,
```
sudo ./tools/workspace/ros/compile_ros_workspace.sh
```
5. Set the environment variable `DAIRLIB_WITH_ROS` to `ON` by Adding to `~/.bashrc`
```
export DAIRLIB_WITH_ROS=ON
```
#### Setup elevation_mapping ros node
Move up to the workspace directory and make a catkin workspace:
```
cd ~/workspace
mkdir catkin_workspace
cd catkin_workspace
```
Add elevation mapping and it's dependencies to src, and install ros prereqs. From `catkin_workspace/`,
```
mkdir src
cd src
git clone git@github.com:anybotics/elevation_mapping.git
git clone git@github.com:anybotics/kindr.git
git clone git@github.com:anybotics/kindr_ros.git
sudo apt install ros-noetic-pcl-ros
```
Build the elevation mapping software:
```
cd ..
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

#### Invariant-EKF
State Estimation for Cassie is done using contact-aided invariant-EKF. `invariant-ekf` is an external repository forked from Ross Hartley's repository of the same name. By default, a pegged version of this forked repository is used i.e. the `bazel` branch of DAIR lab's fork of `invariant-ekf` is automatically downloaded and used. However, to make changes to the files, the [DAIR Lab's fork of invariant-ekf](https://github.com/DAIRLab/invariant-ekf/tree/bazel "DAIR Lab's fork of invariant-ekf") can be cloned as a local repository.

To use local version of `invariant-ekf`, set the environment variable `DAIRLIB_LOCAL_INEKF_PATH`, e.g.
```
export DAIRLIB_LOCAL_INEKF_PATH=/home/user/my-workspace/invariant-ekf
```

### Notes for macOS
1. Be sure to have Xcode 9.0 or later installed with Command Line Tools. If you receive a `clang: error: cannot specify -o when generating multiple output files` message during the build process, re-run `install_prereqs.sh`, and be sure that it runs fully before termination, as this will reconfigure Xcode to work with Drake.


### Build dairlib
Build what you want via Bazel. From `dairlib`,  `bazel build ...` will build the entire project. Drake will be built as an external dependency.
- If you run into ram/cpu limits while building, you can cap the number of threads bazel will use (here we choose `8`) by either:
    - adding `build --jobs=8` to `.bazelrc` 
    - using the `jobs` flag when calling `build` (e.g. `bazel build [target] --jobs=8`)

## Included Modules
A list of included modules

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf

## Docker (experimental)
Docker support is currently experimental. See `install/bionic/Dockerfile` for an Ubuntu Dockerfile. Docker is being used in conjuction with Cirrus Continuous Integration, and should be better supported in the future.
