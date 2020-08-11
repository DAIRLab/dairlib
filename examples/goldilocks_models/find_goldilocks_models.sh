./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=4 --robot_option=1 --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=4 --robot_option=1 --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

cp ../dairlib_data/goldilocks_models/find_models/robot_1/0_* ../dairlib_data/goldilocks_models/find_models/robot_1/nominal_no_constraint_traj/

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=4 --robot_option=1 --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=4 --robot_option=1 --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=1 --max_outer_iter=40 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log


