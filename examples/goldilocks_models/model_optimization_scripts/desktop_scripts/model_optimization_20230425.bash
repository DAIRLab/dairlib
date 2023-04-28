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
#SSBATCH --nodelist=node-2080ti-7
#SSBATCH --exclude=node-2080ti-5

#SBATCH --cpus-per-task=40
#SBATCH --mem-per-cpu=2G

##job setting##
#SBATCH --ntasks=1

if [ ! "$BASH_VERSION" ] ; then
    echo "Please do not use sh to run this script ($0), just execute it directly" 1>&2
    exit 1
fi

set -e

printf "\n\n======================================================================\n"
echo ====== Start running script at `date` =======
printf "======================================================================\n\n\n"

###########model optimization setting##########
# Set robot id and model id
robot=1
model=27 #2, 4

# Set sample size
n_sl=1
n_gi=1
n_du=1
n_tr=1
n_ph=1
n_sm=1
# Set grid parameter
stride_length_center=0.2
ground_incline_center=0
turning_rate_center=0
pelvis_height_center=0.95
swing_margin_center=0.03
stride_length_delta=0.03
ground_incline_delta=0.05
turning_rate_delta=0.125
pelvis_height_delta=0.05
swing_margin_delta=0.02

# main cost weights
Q=0.005    # big weight: 0.1; small weight 0.005
R=0.0002  #
w_joint_accel=0.0001    # big: 0.002; small: 0.0001  # Final weight is w_joint_accel * W_Q

# Testing
zero_ending_pelvis_angular_vel=false
com_at_center_of_support_polygon=false
no_model_update=false  # used to re-evaluate different task while fixing model
is_stochastic=true
heavy_toe=false

# Other parameters
h_step=0.001
final_iter=300

iter_start=1
iter_delta=1

### Some setup
# A check on number of CPUs
#total_cores_needed=$((n_sl*n_gi*n_du*n_tr*n_ph*n_sm))
#if [ "$SLURM_CPUS_PER_TASK" -gt "$((total_cores_needed + 1))" ];
#then printf "Allocated too many cores (%s). This job only need %s.\n" $SLURM_CPUS_PER_TASK $total_cores_needed;
#fi

# Prevent the server from shutting down when using too many cpu cores
n_thread_to_use=0
#n_thread_to_use=$SLURM_CPUS_PER_TASK
#if [ "$HOSTNAME" = "node-2080ti-7" ]; then
#  if [ "$n_thread_to_use" -ge "40" ]; then
#    n_thread_to_use=$(($n_thread_to_use*9/10))
#  fi
#fi

# folder name (Create data folder's name automatically from this bash script's name)
folder_name=$0
folder_name=${folder_name%.bash}  # Get rid of suffix
folder_name=${folder_name%.sh}  # Get rid of suffix
folder_name=${folder_name#examples/goldilocks_models/model_optimization_scripts/desktop_scripts/model_optimization_}  # Get rid of prefix
#if [[ -n "$SLURM_JOB_ID" ]]; then
#  folder_name=`squeue -h -j $SLURM_JOB_ID -o %o`
#  folder_name=${folder_name%.bash}  # Get rid of suffix
#  folder_name=${folder_name#/mnt/beegfs/scratch/yminchen/dairlib/examples/goldilocks_models/model_optimization_scripts/cluster_scripts/model_optimization_on_cluster2_}  # Get rid of prefix
#  folder_name=${folder_name#/mnt/beegfs/scratch/yminchen/dairlib/examples/goldilocks_models/model_optimization_scripts/cluster_scripts/}  # Get rid of prefix
#fi

# Prints
echo robot_option = $robot, rom_option = $model
echo folder_name = $folder_name
echo n_thread_to_use = $n_thread_to_use

directory=../dairlib_data/goldilocks_models/find_models/$folder_name/robot_$robot/

mkdir -p $directory
cp $0 ../dairlib_data/goldilocks_models/find_models/$folder_name/

#cd /scratch/$USER/dairlib

# Note that you need to bazel build the binary, because not all machines/nodes have it. (even though it's the same file path...)
# Build the program
# Bazel threw an error about socket if multiple nodes try to build at the same time, so I use bazel_flag to avoid some of them (this might not prevent in the case where two jobs are run at the same time)
bazel build --jobs=$n_thread_to_use examples/goldilocks_models:find_goldilocks_models
#bazel_flag="/home/${USER}/bazel_is_building"
#while [ -f "$bazel_flag" ]
#do
#  sleep 1
#done
#touch $bazel_flag
#printf "\n\n\n"
#bazel build --jobs=$SLURM_CPUS_PER_TASK examples/goldilocks_models:find_goldilocks_models
#printf "\n\n\n"
#rm $bazel_flag

### Count the lastest iteration (I wrote this becasuse the job can get preempted if run at low QOS
#iter_max=1000  # The code is untested. Just in case we created an infinity loop
#if [ $no_model_update ]; then
#  FILE=_0_c.csv
#else
#  FILE=_theta_y.csv
#fi
#while true
#do
#  FULL_PATH="$directory""$iter_start""$FILE"
#  if [ -f "$FULL_PATH" ]
#  then
#    echo "$FULL_PATH exists."
#    iter_start=$((iter_start+iter_delta))
#  else
#    iter_start=$((iter_start-iter_delta))
#    echo lastest iteration is $iter_start
#    break
#  fi
#
#  if [ "$iter_start" -eq "$iter_max" ]
#  then
#    echo exceed max iter search
#    break
#  fi
#done


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
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== copy files for nomial gaits =====
  # The following line does this (without limit in number of *): cp -n "$directory"0_* "$directory"nominal_no_constraint_traj/
  for i in "$directory"0_* ; do cp -n "$i" "$directory"nominal_no_constraint_traj/; done

  echo ===== evaluate nomial traj \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== copy files for nomial gaits with cubic swing foot constraint =====
  # The following line does this (without limit in number of *): cp -n "$directory"0_* "$directory"nominal_traj_cubic_swing_foot/
  for i in "$directory"0_* ; do cp -n "$i" "$directory"nominal_traj_cubic_swing_foot/; done

  echo ===== evaluate nomial traj with com accel constraint  \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=false --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --com_accel_constraint=true --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== evaluate nomial traj with com accel constraint \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --com_accel_constraint=true --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log


  echo ===== evaluate initial rom \(without snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=1 --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=true \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --delta_iter=$iter_delta \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

else

  # Note that max_inner_iter cannot be too small. Otherwise, the solver never get a chance to find the solution. This messes up the outer loop.
  # Also, we don't use the flag only_update_wrt_main_cost, because it doesn't update well (feels like a bug?)

  # Flags reminder:
  # --N_rerun=0 --max_inner_iter=150 --h_step=2e-5 --beta_momentum=0 --major_optimality_tol=1e-6 --major_feasibility_tol=1e-6 \
  # --only_update_wrt_main_cost=false \

  echo ===== evaluate \(with snopt scaling\) =====
  ./bazel-bin/examples/goldilocks_models/find_goldilocks_models --iter_start=$iter_start --max_outer_iter=$final_iter --snopt_scaling=true --start_current_iter_as_rerun=false \
   --data_folder_name=$folder_name --n_thread_to_use=$n_thread_to_use \
   --Q=$Q --R=$R --w_joint_accel=$w_joint_accel \
   --swing_foot_cublic_spline=true --zero_ending_pelvis_angular_vel=$zero_ending_pelvis_angular_vel --com_at_center_of_support_polygon=$com_at_center_of_support_polygon --no_model_update=$no_model_update --is_stochastic=$is_stochastic --heavy_toe=$heavy_toe \
   --rom_option=$model --robot_option=$robot \
   --delta_iter=$iter_delta \
   --N_sample_sl=$n_sl --N_sample_gi=$n_gi --N_sample_du=$n_du --N_sample_tr=$n_tr --N_sample_ph=$n_ph --N_sample_sm=$n_sm \
   --stride_length_center=$stride_length_center --ground_incline_center=$ground_incline_center --turning_rate_center=$turning_rate_center --pelvis_height_center=$pelvis_height_center --swing_margin_center=$swing_margin_center \
   --stride_length_delta=$stride_length_delta --ground_incline_delta=$ground_incline_delta --turning_rate_delta=$turning_rate_delta --pelvis_height_delta=$pelvis_height_delta --swing_margin_delta=$swing_margin_delta \
   --fix_node_number=true 2>&1 --h_step=$h_step \
   | tee -a "$directory"terminal_log

fi
