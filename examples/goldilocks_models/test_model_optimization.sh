# Set robot id and model id
robot=1
model=4 #2, 4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=1
n_gi=1
n_du=1
n_tr=1

# Other parameters
final_iter=100

# Delete and create a new data folder if specified in the argument
#if [ "$1" = "rm" ]; then
#	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
#	rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
#	mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
#fi

# Build the program
bazel build examples/goldilocks_models:find_goldilocks_models

# Evaluate single traj with ipopt
echo ===== evaluate nomial traj  \(ipopt\) =====
#./bazel-bin/examples/goldilocks_models/find_goldilocks_models  --iter_start=0 --max_outer_iter=0 --start_current_iter_as_rerun=false \
#--ipopt=true \
#--rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --fix_node_number=true 2>&1 | tee ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

echo ===== evaluate nomial traj with swing_foot_cubic_spline \(ipopt\) =====
#./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --start_current_iter_as_rerun=true \
#--swing_foot_cublic_spline=true --ipopt=true \
#--rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

echo ===== evaluate nominal traj with swing_foot_cubic_spline and com accel constraint \(ipopt\) =====
#./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=1 --start_current_iter_as_rerun=true \
#--swing_foot_cublic_spline=true --com_accel_constraint=true --ipopt=true \
#--rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

echo ===== evaluate traj with swing_foot_cubic_spline and com accel constraint \(ipopt\) =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --start_current_iter_as_rerun=true \
--swing_foot_cublic_spline=true --com_accel_constraint=true --ipopt=true \
--rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

