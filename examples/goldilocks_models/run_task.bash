#!/bin/bash

##basic parameters##
#SBATCH --job-name=run_model_opt
#SBATCH --time=20:00
#SBATCH --output=/scratch/$USER/output/slurm-%A_%a.out

##email setting##
#SBATCH --mail-user=$USER@seas.upenn.edu
#SBATCH --mail-type=END
#SBATCH	--mail-type=FAIL

##resources setting##
#SBATCH --partition=posa-compute
#SBATCH --qos=posa-high
#SBATCH --nodes=1
#SBATCH --nodelist=node-2080ti-7

#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=2G

##job setting##
#SBATCH --ntasks=1
#SBATCH --array=0-10%1

cd /scratch/$USER/dairlib
##srun bazel build examples/goldilocks_models:find_goldilocks_models

###########model optimization setting##########
# Set robot id and model id
robot=1
model=2 # 1Drom 2,3   2Drom 0,1  3Drom 4,5
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=1
n_gi=1
n_v=1
n_tr=1

# Set optimization range
# small range
sl_min=0.25
sl_max=0.35
gi_min=-0.15
gi_max=0.15
v_min=0.45
v_max=0.55
tr_min=0
tr_max=0
# large range
# Other parameters
is_grid=true
snopt_scaling=true
N_rerun=1

# Delete and create a new data folder at the beginning
if [ $SLURM_ARRAY_TASK_ID = 0 ]; then
	echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
	rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
	mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
fi

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=$SLURM_ARRAY_TASK_ID --max_outer_iter=$SLURM_ARRAY_TASK_ID --robot_option=$robot --rom_option=$model --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_v=$n_v --N_sample_tr=$n_tr --sl_min=$sl_min --sl_max=$sl_max --gi_min=$gi_min --gi_max=$gi_max --v_min=$v_min --v_max=$v_max --tr_min=$tr_min --tr_max=$tr_max --is_grid_task=$is_grid --fix_node_number=true --snopt_scaling=$snopt_scaling --N_rerun=$N_rerun | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log.txt

wait
echo "Iteration $SLURM_ARRAY_TASK_ID Done"
