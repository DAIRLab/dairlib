## Experiment Instructions

Central repo for C3 and its examples, including:
- https://arxiv.org/abs/2405.08731

This branch/repo is being constantly updated so examples may be unstable.

## Simulated Robot

1. Start the procman script containing a list of relevant processes 
```
bot-procman-sheriff -l franka_sim.pmd
```

2. In the procman window, start the operator processes (meshcat visualizer) using the script `script:start_operator_commands`. Scripts are located in the top bar of the procman window.

3. The meshcat visualizer can be viewed by opening a browser and navigating to `localhost:7000`

4. The examples with the C3 controller can be run using the script `script:c3_mpc`. Note, the task can be changed by changing the `scene_index` in the parameters. More details in [changing the scene](#changing-the-scene). This script spawns three processes:
   - `bazel-bin/examples/franka/franka_sim`: Simulated environment which takes in torques commands from `franka_osc` and publishes the state of the system via LCM on various channels.
   - `bazel-bin/examples/franka/franka_osc_controller`: Low-level task-space controller that tracks task-space trajectories it receives from the MPC
   - `bazel-bin/examples/franka/franka_c3_controller`: Contact Implicit MPC controller that takes in the state of the system and publishes end effector trajectories to be tracked by the OSC.

5. The simulator and controller can be stopped using the script `script:stop_controllers_and_simulators`.
6. A comparison using the sampling based MPC controllers from the [MJMPC controllers](https://github.com/google-deepmind/mujoco_mpc) can be run using `script:mjmpc_with_drake_sim`. This extracts out just the controller portion of the MJMPC code base and runs in on the identical task (scene 1) in the Drake simulator. Instructions to build and configure the standalone MJMPC controllers are a WIP.

## Physical Robot

Hardware instructions updated for Ubuntu 22.04. We are no longer using ROS or ROS2 and instead relying on [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver), which works via LCM. Much thanks to the Drake developers who provided this!

### Installing `drake-franka-driver`

```
git clone https://github.com/RobotLocomotion/drake-franka-driver
cd drake-franka-driver
bazel build ...
```


### Running Experiments

1. Start the procman script containing a list of relevant processes. The primary differences from`franka_sim.pmd` script are the lcm_channels and the drake-franka-driver and corresponding translators to communicate with the Franka via LCM.

   - In the root of dairlib: ``` bot-procman-sheriff -l examples/franka/franka_hardware.pmd ```
   - In the root of drake-franka-driver: ```bot-procman-deputy franka_control``` 

2. In the procman window, start the operator processes (meshcat visualizer and xbox controller) using the script `script:start_operator_commands`. Scripts are located in the top bar of the procman window.

3. The meshcat visualizer can be viewed by opening a browser and navigating to `localhost:7000`

4. The processes, except the C3 controller, can be run using the script `script:start_experiment`. This spawns the following processes:
   - `start_logging.py`: Starts a lcm-logger with an automatic naming convention for the log number.
   - `record_video.py`: Streams all available webcams to a .mp4 file corresponding to the log number.
   - `torque_driver`: `drake-franka-driver` in torque control mode.
   - `franka_driver_`(in/out): communicates with `drake-franka-driver` to receive/publish franka state information and torque commands. This is just a translator between Drake's Franka Panda specific lcm messages and the standardized robot commands that we use. 
   - `bazel-bin/examples/franka/franka_osc_controller`: Low-level task-space controller that tracks task-space trajectories it receives from the MPC
   

5. For safety, start the C3 controller separately after verifying the rest of the processes have started successfully, by manually starting the `franka_c3` process.
6. Using the xbox controller, switch from tracking the teleop commands to the MPC plan by pressing "A".
7. Stop the experiment using `script:stop_experiment`. This also stops logging and recording.


## Changing the scene

We currently have environment descriptions and gains for the following scenes:

The scene can be changed by updating the `scene_index` parameter in BOTH `franka_sim_params.yaml` and `franka_c3_controller_params.yaml`.
The visualizer process must be restarted if changing the scene.

| Scene Index | Description                                                                                                                                   |
|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 0           | No fixed environment features. For testing controlled sliding purely between the end effector and tray + optional objects. Gains still a WIP. |
| 1           | Primary RSS paper example with supports                                                                                                       |
| 2           | Variation on RSS paper example with rotated supports                                                                                          |
| 3           | Additional rotating with wall task for RSS paper                                                                                              |

