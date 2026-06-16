#pragma once
#include "drake/common/yaml/yaml_read_archive.h"

struct HybridMpcOptions {

  int N;
  double dt;

  double w_Q;
  double w_R;
  Eigen::VectorXd q_vector;
  Eigen::VectorXd r_vector;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;

  std::vector<int> quaternion_indices;
  double quaternion_weight;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(q_vector));
    a->Visit(DRAKE_NVP(r_vector));
    a->Visit(DRAKE_NVP(quaternion_indices));
    a->Visit(DRAKE_NVP(quaternion_weight));

    Q = w_Q * q_vector.asDiagonal();
    R = w_R * r_vector.asDiagonal();
  }
};
