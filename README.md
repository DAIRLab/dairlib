# DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction.

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf

## Build Instructions
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
