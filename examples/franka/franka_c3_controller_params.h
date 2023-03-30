#pragma once

#include "solvers/c3_options.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct FrankaC3ControllerParams {
  std::string c3_options_file;
  std::string c3_channel;
  std::string state_channel;
  std::string radio_channel;

  double mu;
  double mu_plate;
  double dt;
  int num_friction_directions;
  std::string franka_model;
  std::string plate_model;
  std::string tray_model;

  int N;
  double w_Q;
  double w_R;
  double w_G;
  double w_U;

  int g_size;
  int u_size;
  std::vector<double> q_vector;

  std::vector<double> r_vector;
  MatrixXd Q;
  MatrixXd R;
  MatrixXd G;
  MatrixXd U;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(c3_channel));
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(mu_plate));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(plate_model));
    a->Visit(DRAKE_NVP(tray_model));

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