# Approximating Global Contact-Implicit MPC via Sampling and Local Complementarity
This is an implementation of our paper currently available on Arxiv.

[[Project webpage](https://approximating-global-ci-mpc.github.io/)] [[Arxiv](https://arxiv.org/abs/2505.13350)] [[Supplemental video](https://youtu.be/rv9n8Uyvoh0)]

## Abstract
> To achieve general-purpose dexterous manipulation, robots must rapidly devise and execute contact-rich behaviors. Existing model-based controllers are incapable of globally optimizing in real-time over the exponential number of possible contact sequences. Instead, recent progress in contact-implicit control has leveraged simpler models that, while still hybrid, make local approximations. However, the use of local models inherently limits the controller to only exploit nearby interactions, potentially requiring intervention to richly explore the space of possible contacts. We present a novel approach which leverages the strengths of local complementarity-based control in combination with low-dimensional, but global, sampling of possible end-effector locations. Our key insight is to consider a contact-free stage preceding a contact-rich stage at every control loop. Our algorithm, in parallel, samples end effector locations to which the contact-free stage can move the robot, then considers the cost predicted by contact-rich MPC local to each sampled location. The result is a globally-informed, contact-implicit controller capable of real-time dexterous manipulation. We demonstrate our controller on precise, non-prehensile manipulation of non-convex objects using a Franka Panda arm.

## Bibtex
```
@article{venkatesh2025approximating,
 title={Approximating Global Contact-Implicit MPC via Sampling and Local Complementarity},
 author={Sharanya Venkatesh* and Bibit Bianchini* and Alp Aydinoglu and William Yang and Michael Posa},
 year={2025},
 journal={arXiv preprint arXiv:2505.13350},
 website={https://approximating-global-ci-mpc.github.io/}
}
```

## Simulation Experiments

1. Start the procman script containing a list of relevant processes.  There's a different one for each demo, currently the 3D jack or planar push T:
```
bot-procman-sheriff -l examples/sampling_c3/jacktoy/franka_sim_jack.pmd  # Jack example
bot-procman-sheriff -l examples/sampling_c3/push_t/franka_sim_t.pmd      # T example
```

2. In the procman window, start the meshcat visualizer by starting the `visualizer` command under the `operator` group.  The meshcat visualizer can be viewed by opening a browser and navigating to `localhost:7000`.

3. The examples with the sampling C3 controller can be run using the script `script:start_experiment_no_logs`. Scripts are located in the top bar of the procman window. This script spawns three processes:
   - `bazel-bin/examples/sampling_c3/franka_sim`: Simulated environment which takes in torques commands from `franka_osc` and publishes the state of the system via LCM on various channels.
   - `bazel-bin/examples/sampling_c3/franka_osc_controller`: Low-level task-space controller that tracks task-space trajectories it receives from the MPC.
   - `bazel-bin/examples/sampling_c3/franka_sampling_c3_controller`: Contact Implicit MPC controller that takes in the state of the system and publishes end effector trajectories to be tracked by the OSC.

4. The simulator and controller can be stopped using the script `script:end_experiment`.


### MJPC Comparison
A comparison using the sampling based MPC controllers from the [MuJoCo MPC controllers](https://github.com/google-deepmind/mujoco_mpc) is currently implemented in `dairlib`'s `jack_mjpc` branch [here](https://github.com/DAIRLab/dairlib/tree/jack_mjpc).  Bringing this code into `sampling_based_c3_public` is WIP.

The MJPC comparison extracts out just the controller portion of the MJPC code base and runs in on the identical task in the Drake simulator. [Our MJPC fork](https://github.com/ebianchi/mujoco_mpc) needs to be installed for this to run properly.  Instructions are WIP.

## Hardware Experiments

### Network Setup
We use a two computer setup with a Franka robot arm:
  1. Computer 1:  Runs sampling C3 controller and [our fork of FoundationPose](https://github.com/dairlab/foundationpose) for object tracking.  Requires a GPU to run FoundationPose.
     - Connected via ethernet to Computer 2.

  2. Computer 2:  Runs our OSC, Franka drivers, and webcam recording.  Requires a realtime kernel for communicating with the Franka.  We rely on [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver), which works via LCM. Much thanks to the Drake developers who provided this!  No ROS/ROS2 involved.

  3. Ethernet connections:
     - Computer 1 <--> Computer 2 with link-local network settings.
     - Computer 2 <--> Franka control box with iPv4 manual network settings matching the Franka IP.

  4. USB connections:
     - RealSense for FoundationPose <--> Computer 1
     - USB webcams for experiment recording <--> Computer 2

### Franka Driver:  Installing `drake-franka-driver`
```
git clone https://github.com/RobotLocomotion/drake-franka-driver
cd drake-franka-driver
bazel build ...
```

### Object Pose Estimation via FoundationPose
[Our fork of FoundationPose](https://github.com/dairlab/foundationpose) handles converting the object pose estimate to world coordinates given access to a camera extrinsic calibration, and it publishes these poses over LCM.  For camera calibration, see [the process documented here](https://github.com/ebianchi/ci_mpc_utils/tree/main?tab=readme-ov-file#camera-calibration).

### Running Experiments

1. Start the procman script containing a list of relevant processes.  The primary differences from the simulation `franka_sim_jack.pmd`/`franka_sim_t.pmd` scripts are the lcm_channels and the drake-franka-driver and corresponding translators to communicate with the Franka via LCM.

   - In the root of dairlib on Computer 1, start the procman sheriff (can use `jack` or `t` examples):
    
    ```
    export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
    bot-procman-sheriff -l examples/sampling_c3/jacktoy/franka_hardware_jack.pmd
    ```

   - In the root of dairlib on Computer 1, start the procman deputy:

    ```
    export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1  # required for python xbox script
    bot-procman-deputy -n sampling_c3_localhost --lcmurl=udpm://239.255.76.67:7667?ttl=1
    ```

   - In the root of drake-franka-driver on Computer 2, start the procman deputy for the Franka driver:    
    
    ```
    bot-procman-deputy -n drake_franka_driver --lcmurl=udpm://239.255.76.67:7667?ttl=1
    ```
 
   - In the root of dairlib on Computer 2, start the procman deputy for the OSC:
   
    ```
    bot-procman-deputy -n franka_control --lcmurl=udpm://239.255.76.67:7667?ttl=1
    ```

2. Start object tracking with FoundationPose.  Can use `jack` or `t` system names.
    ```
    bash docker/run_container.sh
    cd FoundationPose
    python run_live_demo.py --debug=0 --system=jack
    ```

3. In the procman window, start the operator processes (meshcat visualizer and xbox controller) using the script `script:start_operator_commands`. Scripts are located in the top bar of the procman window.  The meshcat visualizer can be viewed by opening a browser and navigating to `localhost:7000`

4. Run the script `script:init_experiment` to slowly move the Franka to an initial configuration.

5. Begin the experiment using the script `script:start_experiment`. This spawns the following processes:
   - `start_logging.py`: Starts a lcm-logger with an automatic naming convention for the log number.
   - `record_video.py`: Streams all available webcams to a .mp4 file corresponding to the log number.
   - `torque_driver`: `drake-franka-driver` in torque control mode.
   - `franka_driver_`(in/out): communicates with `drake-franka-driver` to receive/publish franka state information and torque commands. This is just a translator between Drake's Franka Panda specific lcm messages and the standardized robot commands that we use. 
   - `bazel-bin/examples/sampling_c3/franka_osc_controller`: Low-level task-space controller that tracks task-space trajectories it receives from the MPC.
   - `bazel-bin/examples/sampling_c3/franka_sampling_c3_controller`: Contact Implicit MPC controller that takes in the state of the system and publishes end effector trajectories to be tracked by the OSC.
   - `bazel-bin/examples/sampling_c3/xbox_script`: This gets started with `script:start_operator_commands` but gets restarted with `script:start_experiment` to ensure the experiment starts in teleop mode as a safety precaution.

6. Using the xbox controller, switch from tracking the teleop commands to the MPC plan by pressing "A".  Do this after checking that the plans look reasonable in the visualizer.
7. Stop the experiment using `script:stop_experiment`. This also stops logging and recording.


