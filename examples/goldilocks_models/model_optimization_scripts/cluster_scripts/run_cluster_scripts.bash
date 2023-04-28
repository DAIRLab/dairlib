screen -S job_tracker -d -m python3 examples/goldilocks_models/model_optimization_scripts/cluster_scripts/job_status_tracker.py
screen -S cost_plotter -d -m python3 examples/goldilocks_models/model_optimization_scripts/cluster_scripts/plot_cost_on_cluster.py
screen -S bazel_file -d -m python3 examples/goldilocks_models/model_optimization_scripts/cluster_scripts/monitor_bazel_build_file_age.py
