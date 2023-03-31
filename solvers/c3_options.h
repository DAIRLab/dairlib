#pragma once

#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct C3Options {
  // Hyperparameters
  int admm_iter = 2;    // total number of ADMM iterations   //2
  float rho = 0.1;       // inital value of the rho parameter
  float rho_scale = 3;  // scaling of rho parameter (/rho = rho_scale * /rho) //3
  int num_threads = 0;   // 0 is dynamic, greater than 0 for a fixed count
  int delta_option = 1;  // different options for delta update

  int N;
  double w_Q;
  double w_R;
  double w_G;
  double w_U;

  int g_size;
  int u_size;

  std::vector<double> q_vector;
  std::vector<double> r_vector;

  double mu;
  double mu_plate;
  double dt;
  int num_friction_directions;
  MatrixXd Q;
  MatrixXd R;
  MatrixXd G;
  MatrixXd U;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(admm_iter));
    a->Visit(DRAKE_NVP(rho));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(delta_option));

    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(mu_plate));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(num_friction_directions));

    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_G));
    a->Visit(DRAKE_NVP(w_U));
    a->Visit(DRAKE_NVP(g_size));
    a->Visit(DRAKE_NVP(u_size));
    a->Visit(DRAKE_NVP(q_vector));
    a->Visit(DRAKE_NVP(r_vector));

    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->q_vector.data(), this->q_vector.size());
    Eigen::VectorXd r = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        this->r_vector.data(), this->r_vector.size());

    Q = q.asDiagonal();
    R = r.asDiagonal();
    G = w_G * MatrixXd::Identity(g_size, g_size);
    U = w_U * MatrixXd::Identity(u_size, u_size);
  }
};