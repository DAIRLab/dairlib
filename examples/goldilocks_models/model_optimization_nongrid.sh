echo If you ssh, run this sciprt in the background to prevent the program from being terminated due to disconnection. Sush as: nohup long-running-command \&

# Set robot id and model id
robot=1
model=4 # 1Drom 2,3   2Drom 0,1  3Drom 4,5
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=2
n_gi=1
n_v=1
n_tr=1

# Set optimization range
# small range
sl_min=0.2
sl_max=0.3
gi_min=0
gi_max=0
v_min=0.5
v_max=0.5
tr_min=0
tr_max=0
# large range
#sl_min=-0.3
#sl_max=-0.2
#gi_min=-0.3
#gi_max=0.3
#v_min=0.45
#v_max=0.55
#tr_min=0
#tr_max=0
# Other parameters
is_grid=false
iter_expansion=2
iter_start=0
final_iter=2
snopt_scaling=false
N_rerun=2
momentum=0.5

# Delete and create a new data folder if specified in the argument
if [ "$1" = "rm" ]; then
	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
	rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
	mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
fi


##########################
echo ===== optimize rom$model in small 3D task space \(sl gi v\) with snopt scaling =====
./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=$iter_start --max_outer_iter=$final_iter --robot_option=$robot --rom_option=$model --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --N_sample_tr=$n_tr --sl_min=$sl_min --sl_max=$sl_max --gi_min=$gi_min --gi_max=$gi_max --v_min=$v_min --v_max=$v_max --tr_min=$tr_min --tr_max=$tr_max --is_grid_task=$is_grid --fix_node_number=true --snopt_scaling=$snopt_scaling --N_rerun=$N_rerun --beta_momentum=$momentum --max_num_extending_task_space=$iter_expansion | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log.txt


