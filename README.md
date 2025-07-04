# dairlib
Warning! This is very much "development-level" code and is provided as-is. APIs are likely to be unstable and, while we hope for the documentation to be thorough and accurate, we make no guarantees.

## Current Continuous Integration Status
* `main` branch build and unit tests (Ubuntu Jammy 22.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_jammy&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* `main` branch build and unit tests (Ubuntu Focal 24.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_focal&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* Experimental build against Drake's `master` branch (Jammy): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=drake_master_build&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
## Complete Build Instructions

### 1. Download dairlib
Clone `dairlib` into the your workspace, e.g. "my-workspace/dairlib".
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

In `dairlib/install`, run the `install_prereqs_ubuntu.sh`. Our build process does not currently support MacOS, though it has in the past and likely will in the future.

This option is recommended for users who are not currently editing any source code in Drake itself.

#### Option 2: source install of Drake
If you would like to use your own local install of Drake, likely because you are modifying it, when you build with Bazel you will need to use `bazel build --override_module=drake=/home/user/my-workspace/drake <package you are building>` (using the appropriate directory for your own install). There is no need to build Drake.

### IDE setup
JetBrains IDEs have worked well for us and are available for free to students. For C++ development using the CLion Bazel plugin, see https://drake.mit.edu/clion.html and replace `drake` with `dairlib` in the "Setting up Drake in CLion" section. 

### Notes for macOS

Use the corresponding macOS-specific installation script provided by Drake.

I.e., in the Drake workspace, run:

```
./setup/mac/install_prereqs.sh
```


## Included Modules

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

We have tested that our OSC works for the following robot platforms: Cassie, Franka Panda, and TriFinger. 

### Contact-Implicit MPC (C3)

WIP

See [systems/controllers/c3_controller.cc](systems/controllers/c3_controller.cc)

### Trajectory Optimization (DIRCON)
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf
