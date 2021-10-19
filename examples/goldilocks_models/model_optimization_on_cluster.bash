#!/bin/bash

##basic parameters##
#SBATCH --job-name=run_model_opt
#SBATCH --time=4:00:00
#### commented out. #SSSSBATCH --time=20:00
#SBATCH --output=/scratch/yminchen/output/slurm-%A_%a.out  ##manually specify this directory

##email setting##
#SBATCH --mail-user=yminchen@seas.upenn.edu
#SBATCH --mail-type=END
#SBATCH	--mail-type=FAIL

##resources setting##
#SBATCH --partition=posa-compute
#SBATCH --qos=posa-high
#SBATCH --nodes=1
#### commented out. #SSSSBATCH --nodelist=node-2080ti-7

#SBATCH --cpus-per-task=48
#SBATCH --mem-per-cpu=2G

##job setting##
#SBATCH --ntasks=1
#SBATCH --array=0-100%1

# This script hasn't been fully tested yet.

cd /scratch/$USER/dairlib

# Note that you need to bazel build the binary, because not all machines/nodes have it. (even though it's the same file path...)
# Build the program
bazel build examples/goldilocks_models:find_goldilocks_models


###########model optimization setting##########
# Set robot id and model id
robot=1
model=4 #2, 4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=13
n_gi=3
n_du=1
n_tr=1
n_ph=5

# Other parameters
#final_iter=100

if [ $SLURM_ARRAY_TASK_ID = 0 ]; then
  # Delete and create a new data folder if specified in the argument
  if [ "$1" = "rm" ]; then
    echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/robot_$robot/
    rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
  fi
  mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/
  mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_traj_cubic_swing_foot/

  # Optimize the model
  echo ===== evaluate nomial traj \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=false \
  --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
  --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== copy files for nomial gaits =====
  cp ../dairlib_data/goldilocks_models/find_models/robot_$robot/0_* ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/

  echo ===== evaluate nomial traj \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --swing_foot_cublic_spline=true \
  --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --swing_foot_cublic_spline=true \
  --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== copy files for nomial gaits with cubic swing foot constraint =====
  cp ../dairlib_data/goldilocks_models/find_models/robot_$robot/0_* ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_traj_cubic_swing_foot/

  echo ===== evaluate nomial traj with com accel constraint  \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --com_accel_constraint=true --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== evaluate nomial traj with com accel constraint \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --com_accel_constraint=true --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log


  echo ===== evaluate initial rom \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log

else

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=$SLURM_ARRAY_TASK_ID --max_outer_iter=$SLURM_ARRAY_TASK_ID --snopt_scaling=true --start_current_iter_as_rerun=false \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --fix_node_number=true 2>&1 | tee -a ../dairlib_data/goldilocks_models/find_models/robot_$robot/terminal_log
fi

wait
echo "Iteration $SLURM_ARRAY_TASK_ID Done"

