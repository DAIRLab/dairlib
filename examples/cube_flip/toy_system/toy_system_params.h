#pragma once

#include <Eigen/Dense>
#include "drake/common/yaml/yaml_read_archive.h"

struct ToySystemParams {
  // "open, lqr"
  std::string input_type;
  double time_to_wait;

  Eigen::VectorXd x_init;
  Eigen::VectorXd x_des;

  double sim_rate;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(input_type));
    a->Visit(DRAKE_NVP(time_to_wait));
    a->Visit(DRAKE_NVP(x_init));
    a->Visit(DRAKE_NVP(x_des));
    a->Visit(DRAKE_NVP(sim_rate));

}
};
