# dairlib

## Complete Build Instructions
### Build Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details)

Install Drake from source into `"my-workspace"/drake` http://drake.mit.edu/from_source.html. You do not need to build it, but prerequisites should also be installed. You will need `git` to start.

### Other dependencies
These dependencies are necessary for some advanced visualization and process management. Many examples will work without a full installation of Director or libbot, but (for lab members), these are ultimately recommended. 
1. Install a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2'. The prerequisites installation from Drake should add the proper repo for these.

### Notes for macOS

1. Be sure to have Xcode 9.0 or later installed with Command Line Tools. If you receive a `clang: error: cannot specify -o when generating multiple output files` message during the build process, re-run `install_prereqs.sh`, and be sure that it runs fully before termination, as this will reconfigure Xcode to work with Drake.

### Build dairlib
1. Clone `dairlib` into the same root directory, "my-workspace/dairlib"
```
git clone https://github.com/DAIRLab/dairlib.git
```

2. Build what you want via Bazel. From `dairlib`, `bazel build ...` will build the entire project. Drake will be built as an external dependency.

## Included Modules
A list of included modules

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf
