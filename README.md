# dairlib
Warning! This is very much "development-level" code and is provided as-is. APIs are likely to be unstable and, while we hope for the documentation to be thorough and accurate, we make no guarantees.

## Current Continuous Integration Status
* `main` branch build and unit tests (Ubuntu 22.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_jammy&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* Experimental build against Drake's `master` branch: [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=drake_master_build&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
## Basic Build Instructions

Below are the basic build instructions for the main branch of dairlib without any modifications.
Build variations to include SNOPT/GUROBI/ROS or a specific version of Drake are details in [`install/README.md`](install/README.md)

### 1. Download dairlib
Clone `dairlib` into the your workspace, e.g. "my-workspace/dairlib".
```
git clone https://github.com/DAIRLab/dairlib.git
```

### 2. Install prerequisites (with sudo)
This script just calls the corresponding install_prereqs script from Drake (currently only for ubuntu 22.04)
```
sudo ./install/install_prereqs_jammy.sh
```

### 3. Build with bazel

```
bazel build ...
```
We use bazel as our build system. `bazel build ...` builds everything in dairlib. To build specific binaries, use `bazel build <path>/<to>/<target>`


### 4. LCM and libbot
```
sudo apt install lcm libbot2
```
Installs a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2`. The prerequisites installation should add the proper apt repo for these. If not, add `https://drake-apt.csail.mit.edu/jammy jammy main` to your apt sources

## Other Setup Instructions

### IDE setup
JetBrains IDEs have worked well for us and are available for free to students. For C++ development using the CLion Bazel plugin, see https://drake.mit.edu/clion.html and replace `drake` with `dairlib` in the "Setting up Drake in CLion" section. 

### Notes for macOS
1. Be sure to have Xcode 9.0 or later installed with Command Line Tools. If you receive a `clang: error: cannot specify -o when generating multiple output files` message during the build process, re-run `install_prereqs.sh`, and be sure that it runs fully before termination, as this will reconfigure Xcode to work with Drake.

## Included Modules
A list of included modules

### Cassie Locomotion Controllers

See [examples/Cassie/README.md](examples/Cassie/README.md) for general standing/walking/running/jumping controllers for Cassie in addition to a Cassie simulation with reflected inertia, motor/encoder models.

### Impact-Invariant Control

See [examples/impact_invariant_control/README.md](examples/impact_invariant_control/README.md) for examples specific to impact invariant control

### Operational Space Controller

See [systems/controllers/osc](systems/controllers/osc) for an example of a general Operational Space Controller with support for many task space objectives such as 
- center of mass tracking
- joint space tracking
- task space position tracking
- task space orientation tracking

### Contact-Implicit MPC (C3)

WIP

See [systems/controllers/c3_controller.cc](systems/controllers/c3_controller.cc)

### Trajectory Optimization (DIRCON)
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf
