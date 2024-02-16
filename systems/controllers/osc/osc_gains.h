#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCGains {
  double controller_frequency;

  // costs
  double w_input;
  double w_accel;
  double w_lambda;
  double w_input_reg;
  double w_soft_constraint;
  double impact_threshold;
  double impact_tau;
  double mu;
  std::vector<double> W_accel;
  std::vector<double> W_input_reg;
  std::vector<double> W_lambda_c_reg;
  std::vector<double> W_lambda_h_reg;

  MatrixXd W_acceleration;
  MatrixXd W_input_regularization;
  MatrixXd W_lambda_c_regularization;
  MatrixXd W_lambda_h_regularization;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(controller_frequency));
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_lambda));
    a->Visit(DRAKE_NVP(w_input_reg));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(impact_tau));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(W_accel));
    a->Visit(DRAKE_NVP(W_input_reg));
    a->Visit(DRAKE_NVP(W_lambda_c_reg));
    a->Visit(DRAKE_NVP(W_lambda_h_reg));

    Eigen::VectorXd w_acceleration =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->W_accel.data(),
                                                      this->W_accel.size());
    Eigen::VectorXd w_input_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(this->W_input_reg.data(),
                                                      this->W_input_reg.size());
    Eigen::VectorXd w_lambda_c_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            this->W_lambda_c_reg.data(), this->W_lambda_c_reg.size());
    Eigen::VectorXd w_lambda_h_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            this->W_lambda_h_reg.data(), this->W_lambda_h_reg.size());
    W_acceleration = w_acceleration.asDiagonal();
    W_input_regularization = w_input_regularization.asDiagonal();
    W_lambda_c_regularization = w_lambda_c_regularization.asDiagonal();
    W_lambda_h_regularization = w_lambda_h_regularization.asDiagonal();
  }
};