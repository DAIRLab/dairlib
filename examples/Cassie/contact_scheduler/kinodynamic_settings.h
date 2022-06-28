#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

namespace dairlib {

struct KinodynamicSettings {

  int N;
  int N_per_mode;
  std::vector<double> q_q;
  std::vector<double> q_v;

  MatrixXd Q_q;
  MatrixXd Q_v;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(N_per_mode));
    a->Visit(DRAKE_NVP(q_q));
    a->Visit(DRAKE_NVP(q_v));

    Q_q = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->q_q.data(), 3, 3);
    Q_v = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->q_v.data(), 3, 3);
  }
};
}  // namespace dairlib