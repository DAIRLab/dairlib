Diagram of the OSC Jumping Controller
-
https://docs.google.com/drawings/d/1PnCexul7yWrtmpLJHSUGKIbXj79f9LhUMf0i4xE-Hns/edit?usp=sharing

Workflow 
-
The OSC jumping controller tracks a target trajectory generated through
 trajectory optimization.
 
**Trajectory Generation**
 
The process for generating and processing the trajectory are as follows, the commands can also be run from `bot-procman-sheriff -l cassie_jumping.pmd`:

0. start `bazel-bin/director/drake-director`
1. run `bazel-bin/examples/Cassie/run_dircon_jumping --jumping_height=0.2
 --knot_points=8 --save_filename="<filename>"` or 5.trajectory-optimization dircon_jumping
2. run `bazel-bin/examples/Cassie/visualize_trajectory --folder_path="<foldername> --trajectory_name="<filename> <otherflags>` 
or 5.trajectory-optimization visualize_trajectory (this step is optional, it just allows you to view the saved trajectory at a later time)  
3. run `convert_traj_for_controller --folder_path="<foldername> --trajectory_name="<filename>` or 
5.trajectory-optimization convert_traj_for_controller

**Running the controller and simulator**

The commands to run the controller and simulator can be run from `bot-procman-sheriff -l cassie_jumping.pmd`:

0. start `bazel-bin/director/drake-director --use_builtin_scripts=point_pair_contact`
1. start `bazel-bin/examples/Cassie/visualizer --channel=CASSIE_STATE_SIMULATION`
2. start `bazel-bin/examples/Cassie/run_osc_jumping_controller --traj_name="<filename>" --x_offset=0.095 --delay_time=1.0 --channel_u="CASSIE_INPUT"`
3. start `bazel-bin/examples/Cassie/multibody_sim --time_stepping=true --end_time=5.0 --init_height=1.0`

Flags for visualize_trajectory
-
`visualize_trajectory` has three options for visualizing the saved trajectory from running `run_dircon_jumping`. 
These options are set using the `visualize_mode` flag.
- visualize_mode = 0: Plays the trajectory at `realtime_rate` a single time
- visualize_mode = 1: Plays the trajectory at `realtime_rate` in an endless loop
- visualize_mode = 0: Displays `num_poses` evenly spaced along the trajectory using the multibody pose visualizer
. The code can be modified to display the poses instead at the knot points. 

Mujoco Simulator
-

The Cassie model in MuJoCo (https://github.com/DAIRLab/cassie-mujoco-sim) has slightly different inertia parameters than the Drake model.
Once this issue is figured out, we should be able to use the MuJoCo simulator as well.

Use the `output_contact` branch of `cassie-mujoco-sim` and build it according to the instructions.

To run the simulator to work with the OSC jumping controller:

0. start `bazel-bin/examples/Cassie/run_osc_jumping_controller --traj_name="<filename>" --x_offset=0.095 --delay_time=1.0 --channel_u="CASSIE_INPUT"`
1. start `bazel-bin/examples/Cassie/dispatcher_robot_in --port 25000 --floating_base=true --max_joint_velocity=60 --control_channel_name_1="CASSIE_INPUT"`
or 1.simulated-robot dispatcher-robot-in
2. start `/cassie-mujoco-sim/test/cassiesim -r -s` or 4.other-simulators cassie-mujoco 