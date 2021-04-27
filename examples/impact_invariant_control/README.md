#Examples for Impact Invariant Control
Code documentation including this README is still a work in progress. Please feel free to send any questions about the paper/accompanying code to `yangwill@seas.upenn.edu`.

This folder contains the primary source code for the examples used in:
 
This includes the joint space walking controller for Rabbit and the operational space
jumping controller for Cassie as well as other scripts and dependencies.

Building the source code
- 
We use `bazel` to build our code. To build all of the example scripts, use the command 
`bazel build examples/impact_invariant_control/...`.

Running the scripts
-
We use the process manager, bot-procman (https://github.com/libbot2/libbot2/tree/master/bot2-procman), to organize the scripts. To use procman, use the command 
`bot-procman-sheriff -l examples/impact_invariant_control/impact_invariant_examples.pmd`. Another option is to simply run the compiled binaries located in `bazel-bin`.

The source files for the Cassie examples are located in `examples/Cassie/`.

The controller gains and parameters are located in the `.yaml` files:
- `joint_space_walking_gains.yaml`
- `examples/Cassie/osc_jump/osc_jumping_gains.yaml`

