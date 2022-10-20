# Running Centroidal MPC tracking controller

Controller binary:

`bazel-bin/examples/Cassie/centroidal_mpc_tracking_controller`

Currently the controller runs standalone, it does not need MPC to run in
parallel. This is accomplished by passing in a saved mpc solution through the
gFLAG: `traj_name`.

The default values for gFLAGS have already been set so the controller can be run
just by calling:

`bazel-bin/examples/Cassie/centroidal_mpc_tracking_controller`

The controller communicates with the simulator through LCM. You can start the
simulator by calling

`bazel-bin/examples/Cassie/multibody_sim --init_height=1.0 --toe_spread=0.1 --target_realtime_rate=0.1 --dt=8e-5 --publish_rate=2000 --actuator_delay=0.005`

Visualizing the state of the simulator is done by calling

`bazel-bin/examples/Cassie/visualizer --channel=CASSIE_STATE_SIMULATION`

This will output to drake-director and also output a URL that is running the
Meshcat Visualizer

List of commands
- (optional) `bazel-bin/director/drake-director --use_builtin_scripts=point_pair_contact`
- `bazel-bin/examples/Cassie/visualizer --channel=CASSIE_STATE_SIMULATION`
- `bazel-bin/examples/Cassie/centroidal_mpc_tracking_controller`
- `bazel-bin/examples/Cassie/multibody_sim --init_height=1.0 --toe_spread=0.1 --target_realtime_rate=0.1 --dt=8e-5 --publish_rate=2000 --actuator_delay=0.005`



