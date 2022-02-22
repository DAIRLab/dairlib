#pragma once

struct C3Options {
    // Hyperparameters
    int admm_iter = 10; //total number of ADMM iterations
    int rho = 0.1; //inital value of the rho parameter
    int rho_scale = 2; //scaling of rho parameter (/rho = rho_scale * /rho)
    int num_threads = 0; // 0 is dynamic, greater than 0 for a fixed count
    int timesteps = 500; //number of timesteps for the simulation
};