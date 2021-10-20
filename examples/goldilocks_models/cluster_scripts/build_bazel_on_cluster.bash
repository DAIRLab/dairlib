#!/bin/bash

##basic parameters##
#SBATCH --job-name=build_bazel
#SBATCH --time=2:00:00
#SBATCH --output=/scratch/yminchen/output/slurm.out  ##manually specify this directory
#SBATCH --open-mode=append

##email setting##
#SBATCH --mail-user=yminchen@seas.upenn.edu
#SBATCH --mail-type=END
#SBATCH --mail-type=FAIL

##resources setting##
#SBATCH --partition=posa-compute
#SBATCH --qos=posa-high
#SBATCH --nodes=1
#SBATCH --nodelist=node-2080ti-7

#SBATCH --cpus-per-task=24
#SBATCH --mem-per-cpu=4G

##job setting##
#SBATCH --ntasks=1
#SBATCH --array=0

cd /scratch/$USER/dairlib
#bazel build --jobs=24 examples/goldilocks_models/...
bazel build --jobs=24 examples/goldilocks_models:find_goldilocks_models

#cd /scratch/$USER/drake
#bazel build examples/acrobot:run_passive
#bazel build --jobs=24 ...

echo "End of script."

