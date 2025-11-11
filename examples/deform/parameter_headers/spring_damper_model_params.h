#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct SpringDamperModelParams {
  std::vector<Eigen::VectorXd> vertex_positions;
  std::vector<std::vector<int>> vertex_connections;
  double vertex_scaling;
  Eigen::VectorXd initial_offset;
  double spring_stiffness;
  double damping_coefficient;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(vertex_positions));
    a->Visit(DRAKE_NVP(vertex_connections));
    a->Visit(DRAKE_NVP(vertex_scaling));
    a->Visit(DRAKE_NVP(initial_offset));
    a->Visit(DRAKE_NVP(spring_stiffness));
    a->Visit(DRAKE_NVP(damping_coefficient));
  }
};
