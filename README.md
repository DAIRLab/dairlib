# dairlib

## Build Instructions
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details)
1. Install Drake from source into `"my-workspace"/drake` http://drake.mit.edu/from_source.html. You do not need to build it, but prerequisites should also be installed.
2. Clone `dairlib` into the same root directory, "my-workspace/dairlib"
3. Build what you want via Bazel. From `dairlib`, `bazel build ...` will build the entire project. Drake will be built as an external dependency.
4. Note that some modules and examples will require other dependencies. These will be documented as needed.

## Included Modules
A list of included modules

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction.

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf