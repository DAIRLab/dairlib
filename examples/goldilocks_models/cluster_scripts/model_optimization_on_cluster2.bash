#!/bin/bash

##basic parameters##
#SBATCH --job-name=run_model_opt
#SBATCH --time=48:00:00
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

#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=2G

##job setting##
#SBATCH --ntasks=1
#### commented out. #SSSBATCH --array=0%1
#### commented out. #SSSBATCH --array=0-100%1

if [ ! "$BASH_VERSION" ] ; then
    echo "Please do not use sh to run this script ($0), just execute it directly" 1>&2
    exit 1
fi

echo
echo
echo ======================================================================
echo ====== Start running script at `date` =======
echo ======================================================================
echo
echo

echo Make sure that cpus-per-task is less than total number sample per iter

###########model optimization setting##########
# Set robot id and model id
robot=1
model=16 #2, 4
echo robot_option = $robot, rom_option = $model

# Set sample size
n_sl=1
n_gi=1
n_du=1
n_tr=1
n_ph=1
n_sm=1

# main cost weights
Q=0.1    # big weight: 0.1; small weight 0.005
R=0.0002  #
w_joint_accel=0.0001    # big: 0.002; small: 0.0001  # Final weight is w_joint_accel * W_Q

# Other parameters
final_iter=200
folder_name=

### Some setup
echo folder_name = $folder_name

directory=../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/

cd /scratch/$USER/dairlib

# Note that you need to bazel build the binary, because not all machines/nodes have it. (even though it's the same file path...)
# Build the program
bazel build --jobs=40 examples/goldilocks_models:find_goldilocks_models


### Count the lastest iteration (I wrote this becasuse the job can get preempted if run at low QOS
iter_max=1000  # The code is untested. Just in case we created an infinity loop
iter_start=0
while true
do
  FILE="$directory""$iter_start"_theta_y.csv
  if [ -f "$FILE" ]
  then
    echo "$FILE exists."
    iter_start=$((iter_start+1))
  else
    iter_start=$((iter_start-1))
    echo lastest iteration is $iter_start
    break
  fi

  if [ "$iter_start" -eq "$iter_max" ]
  then
    echo exceed max iter search
    break
  fi
done


### Start optimizing

if [ "$iter_start" -le "1" ]
then

  # Delete and create a new data folder if specified in the argument
  if [ "$1" = "rm" ]; then
    echo Delete and create a new folder dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/
    rm -rf "$directory"
  fi
  mkdir -p "$directory"nominal_no_constraint_traj/
  mkdir -p "$directory"nominal_traj_cubic_swing_foot/

  # Optimize the model
  echo ===== evaluate nomial traj \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== copy files for nomial gaits =====
  cp -n "$directory"0_* "$directory"nominal_no_constraint_traj/

  echo ===== evaluate nomial traj \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== copy files for nomial gaits with cubic swing foot constraint =====
  cp -n "$directory"0_* "$directory"nominal_traj_cubic_swing_foot/

  echo ===== evaluate nomial traj with com accel constraint  \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --com_accel_constraint=true --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj with com accel constraint \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --com_accel_constraint=true --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log


  echo ===== evaluate initial rom \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

  # Note that max_inner_iter cannot be too small. Otherwise, the solver never get a chance to find the solution. This messes up the outer loop.
  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --only_update_wrt_main_cost=true \
   --N_rerun=2 --max_inner_iter=150 \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

else

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=$iter_start --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true \
   --only_update_wrt_main_cost=true \
   --N_rerun=2 --max_inner_iter=150 \
   --rom_option=$model --robot_option=$robot --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm --fix_node_number=true 2>&1 \
   | tee -a "$directory"terminal_log

fi
