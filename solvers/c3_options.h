#pragma once

struct C3Options {
    // Hyperparameters
    int admm_iter = 10; //total number of ADMM iterations
    int rho = 0.1; //inital value of the rho parameter (cartpole 0.1) (gait 1)
    int rho_scale = 2; //scaling of rho parameter (/rho = rho_scale * /rho) (cartpole 2) (gait 1.2)
    int num_threads = 0; // 0 is dynamic, greater than 0 for a fixed count
    int delta_option = 0; //delta update (1 for finger gaiting)
};