# C3-Franka Experiments

[DOCS ARE STILL A WORK IN PROGRESS]

This directory contains the majority of the code required to run the C3-Franka ball rolling experiment.  Many of the instructions in this `README.md` are specific to the Franka setup in GRASP lab.  To run the commands on a separate setup, substitute the correct file paths where necessary.  The documentation is structured as follows:
1. [Installation](#Installation)
2. [Calibration](#Calibration)
3. [Experiments](#Experiments)
4. [Directory Structure](#Directory-Structure)
5. [Relevant Files](#Relevant-Files)
6. [Network Addresses](#Network-Addresses)
7. [FAQ](#FAQ)

## Installation
    
## Calibration
More details about the functionality of each command can be found in the [kuka_vision repository](https://github.com/DAIRLab/kuka-vision)

### Camera Extrinics Calibration
1. Place the calibration board on the table.  Make sure the tag labels match the labels on the table.                                                                                                  
2. Open 4 terminals.  In terminator, this can be done with `Ctrl+Shift+E` or `Ctrl+Shift+O`.  In each terminal:
```
cd ~/apriltag/tagslam_root
source devel/setup.bash
```
3. Move the Pointgrey cameras to a highspeed bus
```
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```
4. Start cameras.
```
roslaunch kuka_vision kuka_online_cam_sync.launch
```
5. Erase existing calibration in `camera_poses.yaml`
```
cp src/kuka_vision/tagslam_config/calib_camera_poses.yaml src/kuka_vision/tagslam_config/camera_poses.yaml
```
6. Start the calibration:
```
roslaunch kuka_vision kuka_online_board_calib.launch
```
7. Open `rviz`.  Click `Add` and add `TF`.  Ensure that all the cameras and tags look like they are in reasonable locations.  Feel free to close `rviz` afterwards.

8. Dump the calibration to the correct location:
```
rosservice call /tagslam/dump
cp ~/.ros/camera_poses.yaml src/kuka_vision/tagslam_config/camera_poses.yaml
```

These steps will write the extrinic camera info into ```src/kuka_vision/tagslam_config/camera_poses.yaml```.  The pose is the position of the camera in the world frame (franka base frame) in meters, and the rotation is the orientation of the camera in the world frame.

### Table Mask Calibration
Ensure that the entire table is visible (not occluded by the ball or the franka)
1. ```cd ~/adam_ws/franka_ws/catkin_ws```
2. ```source devel/setup.bash```
3. ```rosrun franka_vision table_mask_calib.py```
4. Follow the prompts and drag the sliders until you are happy with the table mask.  It doesn't need to be perfect; the table mask is simply used to remove irrelevant edges for the downstream vision algorithm.
5. ```Ctrl+C``` in the terminal and follow the prompts to update the table masks.  The table masks are stored in ```~/adam_ws/franka_ws/catkin_ws/src/franka_vision/calibration```

### Table Height Calibration
Ensure that the correct end effector is attached before proceeding.
1. Turn on franka, unlock joints, and enter FCI mode.  Franka desk can be accessed by visiting `172.16.0.2`.
2. Open procman and run `c.control_node` in group `4.ROS Controllers`.
```
bot-procman-sheriff -l examples/franka_trajectory_following/procman_script_c3.pmd
```
3. Echo the `/franka_state_controller/franka_states` topic and examine the `O_T_EE` field.  This is a 4x4 homogeneous transform matrix organized in column major order.  The center of the EE in the base frame is stored in elements 13-15.  For info about the `O_T_EE`, see the [franka docs](#https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#a193781d47722b32925e0ea7ac415f442).
```
rostopic echo /franka_state_controller/franka_states
```
4. Hit the soft e-stop button and touch the EE to various positions on the table using the teleoperation mode.  Record the EE's z position for each contact.  The EE's radius is 0.0195m.  Back out the table offset accordingly.
5. Enter the table offset (z-level of the table surface in franka's base frame) into the `table_offset` and `model_table_offset` fields in `parameters.yaml`.
6. In the procman, expand group `8.Logging, Utils` and run `e.update_urdf` to update the new table height in all the urdfs.

## Running Experiments
The experimental setup is split across 2 PCs to free up as much compuational power for C3 as possible.  This requires LCM communication to be set up
- Franka PC: handles vision, logging, impedance control, and (optionally) visualization
- C3: handles planning (C3)

Most commands for the experiment can be run through `procman`.  An example photo of the procman from 08/18/2022 is shown here, although present day layout is subject to change.

### Setting up LCM networking
These steps are requried for both sim and hardware experiments in the GRASP lab set up.  If you would like to run all processes, including C3 on the same machine, you can skip these steps since LCM networking won't be used.
1. On Franka PC:
```
sudo ip route add 239.255.76.67 dev eno1
```
2. On C3 PC: `sudo ip route add 239.255.76.67 dev <device>`.  `<device>` can be found by running `supo ip addr`.
3. Test LCM communciation:
```
iperf -c 239.255.76.67 -u -b 20m -f m -i 1 -t 300    # to send from one computer to another
iperf -s -B 239.255.76.67 -u -f m  -i 1              # to receive message from sending computer
```

### Running Experiments in SIM
##### Franka PC
1. `bot-procman-sheriff -l examples/franka_trajectory_following/procman_script_c3.pmd`
2. Start group `1.director`
3. Expand group `7.[SIM] C3`.  `Shift` select commands `b[SIM] franka` to `e.[SIM] sim visualizer` and start the commands.

##### C3 PC
1. `bot-procman-sheriff -l examples/franka_trajectory_following/procman_script_c3.pmd`
2. Start group `1.director` if you wish to visualize on the C3 PC.
3. Run `a.[SIM] C3` in group `7.[SIM] C3`.  Optionally, start `e.[SIM] sim visualizer`.

Currently (08/19/22), visualization is done on Franka PC since the C3 PC has some GPU driver issues; however, if you wish to see the visualizations of the ball path and C3 plan in `drake director`, then visualization must be done on C3 PC since Franka PC is still on Ubunut 18.04.

### Running Experiments in HW
Before beginning, make sure all the calibration steps have been completed.
##### Franka PC
1. Start procman
```
cd ~/adam_ws/dairlib
source ../franka/catkin_ws/devel/setup.bash
bot-procman-sheriff -l examples/franka_trajectory_following/procman_script_c3.pmd
```
2. Start `a.roscore` from group `2.ROS`.
3. Open a new terminal to start the cameras.  Pointgrey cameras take ~1min to warm up.  If you are getting errors, ensure that the cameras are plugged into USB3.0 ports
```
cd ~/apriltag/tagslam_root
source devel/setup.bash
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'  # needs to be run once per bootup
roslaunch kuka_vision kuka_offline_cam_sync_90.launch
```
4. Start group `2.roscore`. During this process, you may see the Pointgrey's drop a few frames as the realsense camera starts up; this is normal.  Click on `b.vision` and ensure that the vision errors look reasonable. If vision change the last 0 flag to 1 in the commmand and rerun to enable visualation. Change the visualization flag back to 0 once you are done debugging.
5. Move the ball to the correct starting location by examining the output of the vision algorithm.
```
rostopic echo /c3/position_estimate
```
6. Start `3.safety`.  This step is `**CRITICAL**`.  The `start_franka_cmds` script will start a safety node if you forget, but you should not rely on this feature.  You should see that safety is waiting for register the `TriggerError` ros service.
7. In the top toolbar of procman, there is a `Scripts` menu.  Click it and start the `start_franka_cmds` script.  This will start all the controllers, logging, and filming.
8. Once the experiment has concluded, run the `stop_franka_cmds` script from the same `Scripts` menu.  This will stop all the controllers in a safe order and begin processing the logs and video frames.

##### C3 PC
1. Start procman
```
cd ~/workspace/dairlib
bot-procman-sheriff -l examples/franka_trajectory_following/procman_script_c3.pmd
```
2. Once the Franka PC has finished running `start_franka_cmds`, start group `5.[HW] C3 Computer`.
3. Once the experiment has concluded, stop `5.[HW] C3 Computer`.

##### Useful Notes
1. Whenever safety triggers, press the soft e-stop, move the robot to a safe configuration if required, and restart the script.
2. If the camera starts missing the ball, you will be able to see this through the errors.  Restart the vision algorithm.
3. Every now and then, vision might throw runtime errors when you restart it.  In this case, just restart again.
4. We will likely be moving away from logging with realsense in the near future since the quality is mediorce and the realtime logging is slow.

##### Common Issues
1. C3 starts, but the robot does not move.  If safety triggered, restart safety and experiment.  If Franka PC is not receiving the `CONTROLLER_INPUT` LCM channel, close the procmans, double check the LCM networking, and restart the whole process.
2. C3 throws mismatched dimension error in contact geoms.  This means that vision did not see the ball.  Restart vision.
3. Robot arm looks like it is moving slowly.  Check C3 output to check if the velocity limit is getting triggered.  If so, increase velocity limit.
4. Franka PC is running out of memory which can lead to failed builds or other weird behavior.

## Directory Structure
This section provides a brief overview of the directory structure for each repository.  For more details about the files in each repository, see [Relevant Files](#Relevant-Files).

### dairlib
- `examples/franka_trajectory_following`: main repository for C3-Franka experiments (ex. parameter files, diagram building scripts, procman files, c3-specific systems, etc)
    - `legacy`: directory containing the old python diagram scripts from May 2022
    - `parameters`: directory containing decent parameters for other balls (out-of-date, should only be used as a rough reference)
    - `robot_property_fingers`: directory containing urdfs, urdf templates (for `update_urdfs` script), meshes, `EE.json`, etc.
    - `scripts`: directory containing logging scripts
    - `systems`: directory containing c3 specific systems
    - `c3_parameters.h`, `parameters.yaml`: parameter related files
    - `run*`: cc files for building and running diagrams
    - `lcm_control_demo`: c3 diagram file
    - `simulate_franka_lcm`: franka simulator
    - `visualizer.cc`: visualizer
    - `procman*`: procman scripts for impedance or c3 experiments
- `bindings/pydairlib/analysis`: directory that implements log post-processing
    - `plot_configs`: directory containing `franka_default_plot.yaml`, the config file for log post-processing
    - `log_plotter_cassie.py`: python script that post processes a log.  Uses utils from other python files in this directory.
- `director/scripts`: contains visualization script, json files, and usage instructions
    - `c3_impedance.json`: visualization for C3-Franka experiments
- `lcmtypes`: directory containing lcm type defintions
    - `lcmt_ball_positions.lcm`: lcm type for reading ball measurements from vision algorithm.
    - `lcmt_c3`: lcm type for c3 experiments
- `solvers`: contains `C3` and `C3MIQP` implementations
- `systems`: contains several key systems that are used as part of the C3-Franka experiments
    - `controllers/impedance_controller.h`: impedance_controller implementation (currently franka specific, but easily modifiable to be more general)
    - `controllers/c3_controller_franka`: `C3MIQP` implementation for C3-Franka experiments.  Velocity limits, gaiting, and experiment initialization are implemented here
    - `ros`: directory containing the relevant systems for Drake<->ROS.
    - `robot_lcm_systems.*`: files implementing sending and receiving classes for LCM messages, including `lcmt_c3` messages.

### franka_ws/catkin_ws/franka_ros
- `franka_control`: directory containing launch and src files for the franka control node.  The safety trigger is implemented in the control node in `franka_control/src/franka_control_node.cpp` (see the `has_error` variable).
- `franka_example_controllers`: directory containing all the example controllers, `move_to_start`, as well as the `drake_impedance_controller`.  For more details on writing controllers, see [Writing your own controller](https://frankaemika.github.io/docs/franka_ros.html#writing-your-own-controller) in the Franka documentation.
    - `include/franka_example_controllers/drake_impedance_controller.h`: header file for the `drake_impedance_controller`.
    - `src/drake_impedance_controller.h`: source file for the `drake_impedance_controller`
- `franka_msgs/srv`: directory containing the declaration of the `TriggerError` service which is used by safety to stop the robot.
- `franka_safety`: directory containing scripts for safety
    - `parameters_safety.yaml`: yaml file containing safety limits
    - `safety.py`: safety script

### franka_ws/catkin_ws/franka_vision
- `src/MaskFinder.py`: `MaskFinder` GUI implementation.  Used by `table_mask_calib.py` and `ball_mask_calib.py` to generate the appropriate masks.
- `src/ball_mask_calib.py`: uses `MaskFinder` to find mask parameters for the ball.  Only required for the mask vision algorithm.
- `src/table_mask_calib.py`: uses `MaskFinder` to create a static binary mask of the table.  Strongly recommended for both the mask and hough vision algorithms.
- `src/pointgrey.py`: vision script that implements both the mask and hough transform methods for the pointgrey cameras.  Takes arguments `<method>` and `<visualization_flag>`.  `<method>` can be either `mask` or `hough`; `visualization_flag>` can be 0 or 1.
- `src/realsense.py`: vision script that implements both the mask and hough transform methods for the realsense camera (has not been maintained for some time).
- `calibration`: directory containing which stores the static binary table masks and the ball mask parameters.  These masks can be modified by calling the appropriate `mask_calib` scripts described above.


## Relevant Files
This section provides a more detailed overview of the relevant files, how they are implemented, and how they should be used.  This section is not yet complete.  For specific file description requests, feel free to email adam.wei@mail.utoronto.ca.
### dairlib

##### examples/franka_trjacetory_following/parameters.yaml

This table describes each parameter and lists the relevant files that use each parameter.  **Note that the relevant files column is not exhaustive.**  To determine all the files that use a parameter, run
```
grep -r <parameter> <dairlib_path>
```

| Parameter      | Type     | Units/Frame |  Functionality | Relevant Files |  Comments |
| :---        |    :----:   |          :---: | :--- | :--- | :------ |
|translational_stiffness | double | N/m | impedance controller translational stiffness | run_impedance_experiment.cc, run_c3_impedance_experiment.cc | |
|rotational_stiffness | double | Nm/rad | impedance controller rotational stiffness | See above | |
|translational_damping_ratio | double | N/A | translational damping ratio for impedance controller: actual damping is computed as 2*translational_damping_ratio * sqrt(translational_stiffness) | See above | |
|rotational_damping | double | Nm*s/rad | rotational damping gains for impedance controller | See above | |
|translational_integral_gain | double | | translational integral gain for impedance controller | impedance_controller.cc | |
|rotational_integral_gain | double | | rotational integral gain for impedance controller | impedance_controller.cc | |
|stiffness_null | double | Nm/rad | nullspace stiffess ratio for rotation and translation | run_impedance_experiment.cc, run_c3_impedance_experiment.cc | |
|damping_null | double | Nm*s/rad | nullspace damping rotation and translation | See above | |
|q_null_desired | VectorXd | rad | nullspace target | See above | |
|moving_offset, pushing_offset | double | m | old parameters for heuristics | impedance_controller.* | |
| mu | double | N/A | friction coefficient for urdfs | urdfs, LCSFactory? | Currently (08-26-22), `update_urdfs.py` does not update this parameter in the urdfs |
| Q_default | double |  | default Q gain for C3 | lcm_control_demo.cc | |
| Q_finger | double |  | Q gain for the end effector xyz | lcm_control_demo.cc | |
| Q_ball_x, Q_ball_y | double |  | Q gain for the balls x and y position respectively | lcm_control_demo.cc | |
| Q_finger_vel, Q_ball_vel | double |  | Q gain for the finger and ball velocity respectively | lcm_control_demo.cc | |
| Qnew_finger, Qnew_ball_x, Qnew_ball_y | double |  | Q gain for the finger xyz, ball_x, and ball_y during the return phase respectively | lcm_control_demo.cc | |
| R | double |  | R gain for the finger xyz, ball_x, and ball_y during the return phase respectively | lcm_control_demo.cc | | G | double |  | G gain for C3 | lcm_control_demo.cc | |
| G | double |  | G gain for C3 | lcm_control_demo.cc | |
| U_default | double | default U cost for C3 | lcm_control_demo.cc | |
|U_pos_vel | double | | U cost for first 19 states in C3 (finger and ball position and velocity) | lcm_control_demo.cc | |
|U_u | double | | U cost for the control inputs (u) in C3) | lcm_control_demo.cc | |
|q_init_finger | VectorXd| m above table surface | desired height of finger above the table surface during the roll phase | lcm_control_demo.cc | Only the last element of this vector is being used.  This parameter could be reduced to a single double |
|q_init_ball_c3 | VectorXd| orientation in quaternions | initial pose of the ball, last 3 elements are not used (overridden after initialization) | lcm_control_demo.cc | Only the first 4 elements of this vector are being used.  This parameter could be reduced to a 4d vector |
|dt | double | s | initial estimate for the period of C3 | lcm_control_demo.cc, c3_controller_franka.* |  |
|velocity_limit | double | m/s | maximum desired velocity returned from C3 | c3_controller_franka.* | Implemented to prevent C3 from returning dangerous set points |
| orientation_degrees | double | degrees | angle of orientaiton target |  c3_controller_franka.* | impedance_controller.* used to use this parameter while we were hacking together some experiments, but generally all the orientation targets should be specified through C3 |
| axis_option | int | | Choose between 1 or 2 to switch between the two axis options for the target orientation. | c3_controller_franka.* | Option 1: rotate away from the target position.  Option 2: rotate towards the center of the ball trajectory.|
| traj_radius | double | m | Radius of the target ball trajectory | c3_state_estimator.cc, simulate_franka_lcm.cc, run_c3_position_experiment.cc, lcm_control_demo.cc, run_c3_impedance_experiment.cc, c3_controller_franka.cc | Also used for computing the initial position of the ball. |
| phase | double | degrees | Starting phase of the circular trajectory | See above | Also used for computing the initial position of the ball. |
| x_c, y_c | double | m in Franka base frame | Center of the target circular trajectory | See above | Also used for computing the initial position of the ball. |
| degree_increment | double | degrees | Controls speed of target trajectory | c3_controller_franka.cc | Only used if enable_adaptive_path is  0, degree_increment must also be positive |
| time_increment | double | s | Desired time to travel degree_increment along the circle | c3_controller_franka.cc | Only used if enable_adaptive_path is  0 |
| hold_order | int |  | Order of the generated trajectory.  0 for a 'discontinuous' trajectory, 1 for a linearly interpolated trajectory.  | c3_controller_franka.cc | Only used if enable_adaptive_path is  0 |
| lead_angle | double | degrees | Controls the distance between the ball's current position and target position | c3_controller_franka.cc | Only used if enable_adaptive_path is  1.  If lead angle is positive, the circle will be drawn CW, otherwise, the circle will be drawn CCW |
| enable_adaptive_path | int | | Controls the way the target trajectory is generated | c3_controller_franka.cc | 0 for set waypoints, 1 to compute way points based on current state |
| ball_radius | double | m | radius of the manipulated sphere | urdfs, c3_state_estimator.cc, simulate_franka_lcm.cc, run_c3_position_experiment.cc, lcm_control_demo.cc, run_c3_impedance_experiment.cc, update_urdfs.py, c3_controller_franka.cc, impedance_controller.cc | `update_urdfs.py` must be called when changing this parameter.  This parameter is also queried by `franka_vision` code.  Usage in `impedance_controller.cc` is legacy. |
| finger_radius | double | m | radius of the end effector (finger) | urdfs,  update_urdfs.py, c3_controller_franka.cc, impedance_controller.cc | `update_urdfs.py` must be called when changing this parameter.  Usage in `impedance_controller.cc` is legacy.|
| EE_offset | VectorXd | m, Franka Frame 7 | position of the center of the finger in Franka frame 7 (last frame) | urdfs, log_plotter_franka.py, update_urdfs.py, c3_controller_franka.cc, impedance_controller.* | `update_urdfs.py` must be called when changing this parameter.|
| table_offset | double | m in Franka base frame |z height of table surface from Franka's world frame (negative for below, positive for above) | urdfs, c3_state_estimator.cc, simulate_franka_lcm.cc, run_c3_position_experiment.cc, run_impedance_experiment.cc, run_c3_impedance_experiment.cc, update_urdfs.py, c3_controller_franka.cc | `update_urdfs.py` must be called when changing this parameter.  This parameter is also queried by `franka_vision` code. |
| model_table_offset | double | m in Franka base frame |z height of table surface from Franka's world frame in C3's world model | urdfs, update_urdfs.py | `update_urdfs.py` must be called when changing this parameter.|
| contact_threshold | double | N | Threshold for feedforward contact force to be added in the impedance controller | impedance_controller.cc | |
| enable_heuristic | int | | Flag to indicate if model moving/pushing heuristics should be used. | impedance_controller.cc | Legacy feature, hasn't been maintained in a while. |
| enable_flag | int | | Flag to indicate if the feedforward force should be added to the controller. | impedance_controller.* |  |
| ball_stddev | double | m | std_dev of noise added to the ball position during simulation.  Assumes guassian noise. | run_state_estimator, c3_controller_franka.cc |  c3_controller_franka.cc uses this parameter through the StateEstimation function which is no longer used.  The noise is now added through the FrankaOutputToBallPosition system instead. |
| roll_phase | double | s | Duration of the roll phase of the gait |  c3_controller_franka.cc, impedance_controller.cc | Usage in impedance_controller.cc has not been maintained in some time (used in the heuristic function). |
| return phase | double | s | Duration of the return phase of the gait |  c3_controller_franka.cc, impedance_controller.cc | Usage in impedance_controller.cc has not been maintained in some time (used in the heuristic function). |
| gait_parameters | VectorXd | m | Parameters that control the rolling gait.  Index 0: controls how far behind the ball the finger should travel at the end of the gait.  Index 1: controls the height of the finger during the first part of the return phase.  Index 2: controls the height of finger during the second part of the return phase.  Index 3: controls the height of the finger during the last part of the return phase. | c3_controller_franka.cc | These parameters tend to make a big difference. |
| dt_filter_length | int | | Length of the moving average filter to compute the C3 dt | c3_controller_franka.* | |
| alpha_p, alpha_v | double | | Exponential filter parameters | c3_controller_franka.* | These parameters aren't actually used.  c3_state_estimator.* uses a hard coded filter value. |



## Network Addresses

## FAQ
1. How do I create a new Drake<->Interface?
Dairlib provides two systems: `ros_subscriber_system` and `ros_publsiher_system`.  The `ros_subscriber_system` subscribes to a rostopic and outputs the ROS messages as `AbstractValue`s; the `ros_publisher_system` takes in a ROS message as a `AbstractValue` and publishes it.  To complete the interface, simply write a system that converts your LCM message type to the desired ROS message type or vise versa, and connect them to the appropriate ros publisher/subcriber systems.

2. Vision algorithm is too slow, how can I speed it up?
The hough algorithm in `franka_vision/src/pointgrey.py` can be tuned to trade off speed for accuracy and precision.  For faster circle detection, try playing with blurring, increasing the `dp` argument in the hough transform, or decreasing the range of ball radii the algorithm is looking for.  Also try recalibrating the table mask.

3. How do I add/remove new parameters to `parameters.yaml`?
Step 1: add/remove the parameter from `parameters.yaml`
Step 2: add/remove the parameter as a member to the `C3Parameters` struct.
Step 3: add/remove the parameter to the `Serialize` member function in `C3Parameters`.
Step 4: Rebuild

4. Webcam has arrived, how do I set it up?
Try using `ffmpeg`.  Ask Will for more details
5. Geom error?
This usually means the cameras could not see the ball, or it thinks the ball is in some physically infeasible spot (ex. in the robot)

6. C3 target in sim is jumping everywhere?
This can happen if C3 is running on both the Franka and the C3 PC.

7. LCM messages are not being sent across the network.
Try the following steps until it works. Restart procmans, sudo ip route add command, test LCM connection through terminal, check memory availability on Franka computer, restart computers, ask Cassie guys for help 😊.

8. Dairlib won't build.
Check if the computer has enough memory (Franka PC is low on memory).  Also check the BAZEL files and the `#define ROS` statements and set them appropriately.  Since only some GRASP computers have ROS, the BAZEL file tends to get editted frequently which might lead to build issues for other users.  In the future, we should look into build flags to have a more streamlined build process.

9. When should I restart the safety script?
If you are unsure, or if you are reading this question, that means you should probably restart safety.  Serious answer, you should restart safety whenever you get an error related to the `TriggerError` service, but it never hurts to restart safety more frequently anyways.

10. How come I can't watch the videos from the logs?
For some reason, you can't watch the mp4 files from the harddrive.  Send the video over slack, or copy it into a directory outside the harddrive.

11. The pointgrey camera's drop frames when realsense starts.  Is this normal?
Yes.

12. When should I recalibrate cameras?
When you get `Discarding measurement` errors from 1 or more cameras.
