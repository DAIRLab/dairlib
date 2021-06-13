Simulation versus hardware comparison for impacts
-

Basic instructions for processing the raw lcm logs and initializing the simulation from those logs.

Additional dependencies
- I use cnpy: https://github.com/rogersce/cnpy to convert `.npy` files to Eigen matrices
    - to use cnpy
        - clone the repo in the same parent directory as dairlib
        - create a `BUILD.bazel` file in that repo with contents: 
               
               cc_library(
                 name = "cnpy",
                 srcs = ["cnpy.cpp"],
                 hdrs = ["cnpy.h"],
                 visibility = ["//visibility:public"],
               )
        - set the environment variable `CNPY_LOCAL_PATH`  
               

Key files:
-
- `bindings/pydairlib/analysis_scripts/convert_log_to_trajectory_data.py`
    - script to convert lcmlogs to state/input/time .npy files
    - The converted lcmlogs are already in the data folder on the google drive, but the script is provided in case we 
    want to save/load additional values  
- `bindings/pydairlib/analysis_scripts/log_plotter_cassie.py`
    - do it all plotting script. This script contains all the miscellaneous plotting functionality to analyze/debug an 
    lcmlog from a cassie experiment. Most parts of this script are irrelevant, the functions that will probably be 
    useful are:
        - `plot_state()`: plots state vs time and inputs vs time.
        - `plot_ii_projection()`: plots joint velocities in a small window around the impact time
        - `plot_feet_positions()`: plots the estimated position and velocity of each contact point for debugging 
        purposes   
    - usage:
        - `bazel-bin/bindings/pydairlib/analysis_scripts/log_plotter_cassie` `<path_to_log>` `<controller_channel>`
        - `path_to_log` should be the absolute path to the lcmlog
        - `controller_channel` for jumping is `OSC_JUMPING` for hardware logs and `CASSIE_INPUT` for simulation logs. 
- `bindings/pydairlib/analysis_scripts/impact_invariant_scripts.py`
    - library that contains `plot_ii_projection()`
    - modify this to change the time window
- `examples/Cassie/multibody_sim_playback.cc`
    - standalone Cassie simulation that initializes the simulation from time `start_time` from the specified log
    - state and input data are broadcast using lcm
        - to record and then analyze this data, use `lcm-logger` and plot the log using `log_plotter_cassie`
        
Data files:
-
- The datafiles are organized by date and log number: `mm/dd/yy/<filetype> + log_number`. 
- lcmlogs contains all the lcm messages broadcast by the state estimator and controller
- controller gains are stored in `.yaml` files
- `commit_tag` stored the git commit hash and any local diffs from that commit
- The successful jumping experiments are:
    - logs 12-15 from 01_27_21
    - logs 27-34 from 02_12_21
    - the robot was unable to complete the jump in the other logs from these dates or the parameters were still being 
    tuned
    
Typical process to compare the hardware data and sim data
- start the `lcm-logger` to log the data in the chosen data folder
- run `bazel-bin/examples/Cassie/multibody_sim_playback` with the desired parameters:
    - example: `bazel-bin/examples/Cassie/multibody_sim_playback --npy_num=28 --start_time=30.595`
- plot the state/input data from the hardware data using `log_plotter_cassie`
- plot the state/input data from the simulated data using `log_plotter_cassie`


