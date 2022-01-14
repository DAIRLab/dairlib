# Set robot id and model id
robot=1
model=4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=5
n_gi=1
n_du=1
n_tr=1
n_ph=5

# Other parameters
final_iter=1
folder_name=

# Multithreading
threads=8

# Delete and create a new data folder if specified in the argument
if [ "$1" = "rm" ]; then
	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/
	rm -rf ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/
fi
mkdir -p ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/nominal_no_constraint_traj/
mkdir -p ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/nominal_traj_cubic_swing_foot/

# Build the program
bazel build examples/goldilocks_models:find_goldilocks_models

# Optimize the model
echo ===== evaluate nomial traj \(without snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=false \
 --data_folder_name=$folder_name \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== evaluate nomial traj \(with snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== copy files for nomial gaits =====
cp -n ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/0_* ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/nominal_no_constraint_traj/

echo ===== evaluate nomial traj \(without snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== evaluate nomial traj \(with snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== copy files for nomial gaits with cubic swing foot constraint =====
cp -n ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/0_* ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/nominal_traj_cubic_swing_foot/

echo ===== evaluate nomial traj with com accel constraint  \(without snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --com_accel_constraint=true --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== evaluate nomial traj with com accel constraint \(with snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --com_accel_constraint=true --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log


echo ===== evaluate initial rom \(without snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false \
 --data_folder_name=$folder_name \
 --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log

echo ===== evaluate \(with snopt scaling\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --n_thread_to_use=$threads --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true \
 --data_folder_name=$folder_name \
 --swing_foot_cublic_spline=true \
 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 \
 | tee -a ../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/terminal_log



#### Testing method_to_solve_system_of_equations ####
#echo ===== evaluate \(with snopt scaling\) =====
#./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true \
# --N_rerun=0 --method_to_solve_system_of_equations=6 --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log






######################################
# Re-evalutate nomial traj
# echo ===== evaluate nomial traj \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true 2>&1 | tee ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# echo ===== evaluate nomial traj \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
#echo ===== evaluate nomial traj \(with ipopt\) =====
#./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --ipopt=true --n_thread_to_use=1 --start_current_iter_as_rerun=true \
#--rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################
# Evaluate initial rom only
# echo ===== evaluate initial rom \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=true 2>&1 | tee ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# echo ===== evaluate initial rom \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# Testing
# echo ===== evaluate initial rom \(without snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
# echo ===== evaluate initial rom \(with snopt scaling\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################
# Testing. Use ipopt
# echo ===== Use ipopt =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=1 --max_outer_iter=1 --ipopt=true --start_current_iter_as_rerun=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
######################################
# echo ===== evaluate nomial traj with com accel constraint \(ipopt\) =====
# ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true --com_accel_constraint=true --ipopt=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

