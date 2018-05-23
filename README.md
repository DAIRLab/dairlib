# DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction.

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf

Two approaches to building the project are recommended. See http://drake.mit.edu/ for instructions on building Drake.

## Build Instructions (Bazel)
* This approah uses a local copy of the Drake source code. See https://github.com/RobotLocomotion/drake-shambhala for other approaches to building against Drake.
* Download Drake and DIRCON source into the same root directory
```git clone https://github.com/RobotLocomotion/drake.git```
* Download DIRCON
```git clone https://github.com/DAIRLab/DIRCON.git```
* Build DIRCON (will build Drake as an external project)
```
cd DIRCON
bazel build ...
```

## Build Instructions (CMake)
* Install Drake as a binary
* Download DIRCON 
```git clone https://github.com/DAIRLab/DIRCON.git```
* Configure Cmake
```
mkdir build && cd build
cmake -Ddrake_DIR=/opt/drake/lib/cmake/drake ../
```
* Build DIRCON
```
make
```
