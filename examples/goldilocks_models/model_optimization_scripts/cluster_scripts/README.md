To automate model optimization (or open-loop evaluation) on the cluster,
1. run the bash script with sbatch
   e.g. sbatch examples/goldilocks_models/model_optimization_scripts/cluster_scripts/20221209_explore_task_boundary_2D--rom27_big_range_bigger_step_size_5e-3_torque_weight_dominate_com_center.bash
        (note that some jobs such as the above example require prior files, so we need to create the folder and copy files manually first)
2. add the bash script name to job_status_tracker.py to let it babysit the job
   e.g. nonstop_sbatch_script = []  # re-submit scripts if added. Only file name is required (don't need directory)
        nonstop_sbatch_script.append("20221209_explore_task_boundary_2D--rom27_big_range_bigger_step_size_5e-3_torque_weight_dominate_com_center.bash")
3. run run_cluster_scripts.bash (this runs all necessary scripts)
   bash examples/goldilocks_models/model_optimization_scripts/cluster_scripts/run_cluster_scripts.bash 


