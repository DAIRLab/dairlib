#pragma once

struct C3Options {
  // Hyperparameters
  int admm_iter = 3;    // total number of ADMM iterations
  float rho = 0.1;       // inital value of the rho parameter
  float rho_scale = 3;  // scaling of rho parameter (/rho = rho_scale * /rho)
  int num_threads = 2;   // 0 is dynamic, greater than 0 for a fixed count
  int delta_option = 1;  // different options for delta update
};