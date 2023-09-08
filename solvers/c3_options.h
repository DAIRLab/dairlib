#pragma once

struct C3Options {
  // Hyperparameters
  int admm_iter = 3;    // total number of ADMM iterations    //3   //old one 2
  float rho = 0.1;       // inital value of the rho parameter
  float rho_scale = 2.2;  // scaling of rho parameter (/rho = rho_scale * /rho)  //2.2 //old one 3
  int num_threads = 2;   // 0 is dynamic, greater than 0 for a fixed count
  int delta_option = 1;  // different options for delta update
};