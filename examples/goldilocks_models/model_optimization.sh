echo If you ssh, run this sciprt in the background to prevent the program from being terminated due to disconnection. Sush as: nohup long-running-command &

# Set robot id and model id 
robot=1
model=4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=25
n_gi=5
n_v=1

# Delete and create a new data folder if specified in the argument
if [ "$1" = "rm" ]; then
	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
	rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
	mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
fi

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

cp ../dairlib_data/goldilocks_models/find_models/robot_1/0_* ../dairlib_data/goldilocks_models/find_models/robot_1/nominal_no_constraint_traj/

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=40 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

