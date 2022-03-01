#pragma once

struct C3Options {
    // Hyperparameters
    int admm_iter = 10; //total number of ADMM iterations
    int rho = 1; //inital value of the rho parameter (cartpole 0.1) (gait 1)
    int rho_scale = 1.2; //scaling of rho parameter (/rho = rho_scale * /rho) (cartpole 2) (gait 1.2)
    int num_threads = 0; // 0 is dynamic, greater than 0 for a fixed count
    int timesteps = 500; //number of timesteps for the simulation
};