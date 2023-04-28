#!/bin/bash
#
#SBATCH --job-name=bazel_build
#SBATCH --output=../bazel_build.txt
#
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=17
#SBATCH --time=2:00:00
#SBATCH --mem-per-cpu=2G
#SBATCH --nodes=1
#
#SBATCH --array=1

bazel build examples/goldilocks_models:find_goldilocks_models 

