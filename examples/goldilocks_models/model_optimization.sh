echo If you ssh, run this sciprt in the background to prevent the program from being terminated due to disconnection. Sush as: nohup long-running-command \&

# Set robot id and model id
robot=1
model=0 #2, 4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=1
n_gi=1
n_v=1

# Other parameters
final_iter=1

# Delete and create a new data folder if specified in the argument
if [ "$1" = "rm" ]; then
	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
	rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
	mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
fi

# echo ===== evaluate nomial traj \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

# echo ===== evaluate nomial traj \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

# echo ===== copy files for nomial gaits =====
# cp ../dairlib_data/goldilocks_models/find_models/robot_$robot/0_* ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/


# echo ===== evaluate nomial traj with com accel constraint  \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true --com_accel_constraint=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

# echo ===== evaluate nomial traj with com accel constraint \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true --com_accel_constraint=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log


# echo ===== evaluate initial rom \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

# echo ===== evaluate \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log






######################################
# Re-evalutate nomial traj
# echo ===== evaluate nomial traj \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true | tee ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# echo ===== evaluate nomial traj \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################
# Evaluate initial rom only
echo ===== evaluate initial rom \(without snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=true | tee ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
echo ===== evaluate initial rom \(with snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# Testing
# echo ===== evaluate initial rom \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# echo ===== evaluate initial rom \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################
# Testing. Use ipopt
# echo ===== Use ipopt =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=1 --max_outer_iter=1 --ipopt=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################

# echo ===== evaluate nomial traj with com accel constraint \(ipopt\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true --com_accel_constraint=true --ipopt=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

