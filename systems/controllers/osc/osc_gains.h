#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCGains {
  // costs
  double w_input;
  double w_input_reg;
  double w_accel;
  double w_soft_constraint;
  double impact_threshold;
  double mu;
  std::vector<double> W_accel;
  std::vector<double> W_input_reg;
  std::vector<double> W_lambda_reg;

  MatrixXd W_acceleration;
  MatrixXd W_input_regularization;
  MatrixXd W_lambda_regularization;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_input_reg));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(W_accel));
    a->Visit(DRAKE_NVP(W_input_reg));
    a->Visit(DRAKE_NVP(W_lambda_reg));

    Eigen::VectorXd w_acceleration =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->W_accel.data(),
                                                      this->W_accel.size());
    Eigen::VectorXd w_input_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->W_input_reg.data(),
                                                      this->W_input_reg.size());
    Eigen::VectorXd w_lambda_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            this->W_lambda_reg.data(), this->W_lambda_reg.size());
    W_acceleration = w_acceleration.asDiagonal();
    W_input_regularization = w_input_regularization.asDiagonal();
    W_lambda_regularization = w_lambda_regularization.asDiagonal();
  }
};