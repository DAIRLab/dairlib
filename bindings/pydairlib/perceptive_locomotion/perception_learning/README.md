##Sandbox for learning perceptive locomotion for Cassie

This is an environment to support som initial work doing perceptive learning on 
Cassie. For questions about infrastructure and setup, reach out to 
@Brian-Acosta. 

###Ground rules:
1. To keep this workspace tidy, any temporary files like logs, data, etc. should be
saved to the tmp subdirectory, which is already part of the global 
dairlib .gitignore on this branch.

###Why not use the cassie gym project you already set up, Brian?
That project is a rat's nest with all sorts of weird hard-coded dependencies 
that make reusing the simulation and controller blocks kind of a nightmare. 
My goal with this project is to keep everything a little-bit more modular and 
reusable by exposing simulator and controller configuration through external 
yaml files, and multiple options for input/output ports to allow for adding 
intermediate blocks like state estimation, elevation mapping, etc. 

