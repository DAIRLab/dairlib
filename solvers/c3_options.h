#pragma once

#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_read_archive.h"

struct C3Options {
  // Hyperparameters
  int admm_iter = 2;    // total number of ADMM iterations   //2
  float rho = 0.1;       // inital value of the rho parameter
  float rho_scale = 3;  // scaling of rho parameter (/rho = rho_scale * /rho) //3
  int num_threads = 0;   // 0 is dynamic, greater than 0 for a fixed count
  int delta_option = 1;  // different options for delta update
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(admm_iter));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(delta_option));

  }
};