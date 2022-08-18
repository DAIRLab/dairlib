# C3-Franka Experiments

[DOCS ARE STILL A WORK IN PROGRESS]

This directory contains the majority of the code required to run the C3-Franka ball rolling experiment.  The documentation will be structured as follows:
1. [Installation](#Installation)
2. [Calibration/Experiments](#Calibration/Experiments)
3. [Directory Structure](#Directory-Structure)
4. [Relevant Files](#Relevant-Files)

## Installation
    
## Calibration/Experiments
Note: These instructions are specific to running the experiment on the C3-Franka PCs in the GRASP lab.  To run the experiment on other another setup, substitute the appropriate directory paths as required.

### Camera Extrinsics and Table Mask Calibration
More details about the functionality of each command can be found in the [kuka_vision Repository](https://github.com/DAIRLab/kuka-vision)

#### Extrinics Calibration
1. Place the calibration board on the table.  Make sure the tag labels match the labels on the table.                                                                                                  
2. Open 4 terminals.  In terminator, this can be done with `Ctrl+Shift+E` or `Ctrl+Shift+O`
3. cd into `~/april_tag/tagslam_config` and source the 'devel/setup.bash' in all terminals
```
cd ~/april_tag/tagslam_config
source 'devel/setup.bash'
```
4. In terminal 1, run `roscore`
5. In terminal 2, move the Pointgrey cameras to a highspeed bus
```
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```
6. In terminal 2, start the cameras and wait for them to warm up.
```
roslaunch kuka_vision kuka_online_cam_sync.launch
```
7. In terminal 3, "reset" `camera_poses.yaml`
```
cp src/kuka_vision/tagslam_config/calib_camera_poses.yaml src/kuka_vision/tagslam_config/camera_poses.yaml
```
8. In terminal 3, start the calibration
```
roslaunch kuka_vision kuka_online_board_calib.launch
```
9. In terminal 4, open `rviz`.  Click `Add` and add `TF`.  Ensure that all the cameras and tags look like they are in reasonable locations.  Close rviz afterwards

10. In terminal 4, dump the calibration info to the correct location.  Afterwards, close all terminals.
```
rosservice call /tagslam/dump && cp ~/.ros/camera_poses.yaml src/kuka_vision/tagslam_config/camera_poses.yaml
```

These steps will write the extrinic camera info into ```src/kuka_vision/tagslam_config/camera_poses.yaml```.  The pose is the position of the camera in the world frame (franka base frame) in meters, and the rotation is the orientation of the camera in the world frame.

#### Table Mask Calibration
Ensure that the entire table is visible (not occluded by the ball or the franka)
1. ```cd ~/adam_ws/franka_ws/catkin_ws```
2. ```source devel/setup.bash```
3. ```rosrun franka_vision table_mask_calib.py```
4. Follow the prompts and drag the sliders until you are happy with the table mask.  It doesn't need to be perfect; the table mask is simply used to remove irrelevant edges for the downstream vision algorithm.
5. ```Ctrl+C``` in the terminal and follow the prompts to update the table masks.  The table masks are stored in ```~/adam_ws/franka_ws/catkin_ws/src/franka_vision/calibration```

### Running Experiments
The experimental setup is split across 2 PCs to free up as much compuational power for C3 as possible.
- Franka PC: handles vision, logging, impedance control, and (optionally) visualization
- C3: handles planning (C3)

All commands for the experiment can be run through `procman` except the Pointgrey cameras which must be started in a separate terminal.  An example photo of the procman from 08/18/2022 is shown here, although present day layout is subject to change.

                                                                                                  
### Running Experiments: C3 PC

## Directory Structure

## Relevant Files
