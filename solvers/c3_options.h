#pragma once

struct C3Options {
    // Put any hyperparameters or other options here
    int admm_iter = 10;
    int rho = 0.1;
    int rho_scale = 2;
    int num_threads = 0; // 0 is dynamic, greater than 0 for a fixed count
};