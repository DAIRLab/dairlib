#!/bin/bash

bazel build examples/goldilocks_models/evaluation_scripts:eval_rom_dynamics_function

# `&&` is used to not run python script if eval_rom_dynamics_function threw error (e.g. matrices contain NaN)
bazel-bin/examples/goldilocks_models/evaluation_scripts/eval_rom_dynamics_function && python3 examples/goldilocks_models/evaluation_scripts/visualize_rom_dynamics_function/plot_rom_dynamics_function.py
