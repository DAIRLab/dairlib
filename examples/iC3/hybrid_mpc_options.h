#pragma once
#include "drake/common/yaml/yaml_read_archive.h"
#include "c3/multibody/lcs_factory_options.h"

struct HybridMpcOptions {

  std::string osqp_settings;
  std::string object_model;

  double w_Q;
  double w_R;
  double w_S;
  double w_G; // Penalty on epsilon (complementarity slack variable)
  double accel_cost; 

  Eigen::VectorXd q_vector;
  Eigen::VectorXd r_vector;
  Eigen::VectorXd s_vector;
  Eigen::VectorXd g_vector;

  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd S;
  Eigen::MatrixXd G;

  Eigen::VectorXd lambda_threshold;
  Eigen::VectorXd eta_threshold;

  std::vector<int> quaternion_indices;
  double quaternion_weight;

  bool use_nominal_lcs;
  int transform_update_frequency; 

  c3::LCSFactoryOptions lcs_factory_options;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(osqp_settings));
    a->Visit(DRAKE_NVP(object_model));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_S));
    a->Visit(DRAKE_NVP(w_G));
    a->Visit(DRAKE_NVP(q_vector));
    a->Visit(DRAKE_NVP(r_vector));
    a->Visit(DRAKE_NVP(s_vector));
    a->Visit(DRAKE_NVP(g_vector));
    a->Visit(DRAKE_NVP(quaternion_indices));
    a->Visit(DRAKE_NVP(quaternion_weight));
    a->Visit(DRAKE_NVP(use_nominal_lcs));
    a->Visit(DRAKE_NVP(lambda_threshold));
    a->Visit(DRAKE_NVP(eta_threshold));
    a->Visit(DRAKE_NVP(accel_cost));
    a->Visit(DRAKE_NVP(transform_update_frequency));

    a->Visit(DRAKE_NVP(lcs_factory_options));

    Q = w_Q * q_vector.asDiagonal();
    R = w_R * r_vector.asDiagonal();
    S = w_S * s_vector.asDiagonal();
    G = w_G * g_vector.asDiagonal();

  }
};
