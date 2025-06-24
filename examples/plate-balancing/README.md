# Plate-Balancing

This repository contains code for balancing a tray on the Franka Emika Panda robot, using Contact-Implicit Model Predictive Control (C3-MPC). The controller leverages contacts with the environment to perform manipulation tasks.

https://arxiv.org/abs/2405.08731

## Simulated Robot

1. Start the procman script from the workspace directory (dairlib). It contains a list of relevant processes 
```
bot-procman-sheriff -l examples/plate-balancing/procman/plate_balancing_simulation.pmd
```

2. In the procman window, start the operator processes (meshcat visualizer) using the script `script > start_operator_commands`. Scripts are located in the top bar of the procman window.

3. The meshcat visualizer can be viewed by opening a browser and navigating to `localhost:7000`

4. The examples with the C3 controller can be run using the script `script > c3_mpc`. Note, the task can be changed by changing the `scene_index` in the parameters. More details in [changing the scene](#changing-the-scene). This script spawns three processes:
   - `bazel-bin/examples/plate-balancing/run_simulation`: Simulated environment which takes in torques commands from `franka_osc` and publishes the state of the system via LCM on various channels.
   - `bazel-bin/examples/plate-balancing/run_osc_controller`: Low-level task-space controller that tracks task-space trajectories it receives from the MPC
   - `bazel-bin/examples/plate-balancing/run_c3_controller`: Contact Implicit MPC controller that takes in the state of the system and publishes end effector trajectories to be tracked by the OSC.

5. The simulator and controller can be stopped using the script `script:stop_controllers_and_simulators`.
6. A comparison using the sampling based MPC controllers from the [MJMPC controllers](https://github.com/google-deepmind/mujoco_mpc) can be run using `script:mjmpc_with_drake_sim`. This extracts out just the controller portion of the MJMPC code base and runs in on the identical task (scene 1) in the Drake simulator. Instructions to build and configure the standalone MJMPC controllers are a WIP.

## Changing the scene

We currently have environment descriptions and gains for the following scenes:

The scene can be changed by updating the `scene_index` parameter in  `examples/plate-balancing/config/plate_balancing_config.yaml`.
The visualizer process must be restarted if changing the scene.

| Scene Index | Description                                                                                                                                   |
|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 0           | No fixed environment features. For testing controlled sliding purely between the end effector and tray + optional objects. Gains still a WIP. |
| 1           | Primary RSS paper example with supports                                                                                                       |
| 2           | Variation on RSS paper example with rotated supports                                                                                          |
| 3           | Additional rotating with wall task for RSS paper                                                                                              |

