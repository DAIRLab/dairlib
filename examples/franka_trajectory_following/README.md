# C3-Franka Experiments

[DOCS ARE STILL A WORK IN PROGRESS]

This directory contains the majority of the code required to run the C3-Franka ball rolling experiment.  Many of the instructions in this `README.md` are specific to the Franka setup in GRASP lab.  To run the commands on a separate setup, substitute the correct file paths where necessary.  The documentation is structured as follows:
1. [Installation](#Installation)
2. [Calibration](#Calibration)
3. [Experiments](#Experiments)
4. [Directory Structure](#Directory-Structure)
5. [Relevant Files](#Relevant-Files)
6. [FAQ](#FAQ)

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
source ../franka_ws/catkin_ws/devel/setup.bash
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

### franka_ws/catkin_ws


## Relevant Files

## FAQ
TODO: more questions here
1. How do I create a new Drake<->Interface?
2. Vision algorithm is too slow, how can I speed it up?
3. How do I add/remove new parameters to `parameters.yaml`?
4. Webcam has arrived, how do I set it up?
5. Geom error?
6. C3 target in sim is jumping everywhere?
7. LCM messages are not being sent across the network.
