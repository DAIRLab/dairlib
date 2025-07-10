#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerImpedanceControllerParams {
  std::string trifinger_model;
  std::string fingertip_0_name;
  std::string fingertip_120_name;
  std::string fingertip_240_name;
  Eigen::VectorXd min_fingertips_delta_position;
  Eigen::VectorXd max_fingertips_delta_position;
  std::vector<double> KpFingertip0;
  std::vector<double> KdFingertip0;
  std::vector<double> KpFingertip120;
  std::vector<double> KdFingertip120;
  std::vector<double> KpFingertip240;
  std::vector<double> KdFingertip240;
  Eigen::MatrixXd Kp_fingertip_0;
  Eigen::MatrixXd Kd_fingertip_0;
  Eigen::MatrixXd Kp_fingertip_120;
  Eigen::MatrixXd Kd_fingertip_120;
  Eigen::MatrixXd Kp_fingertip_240;
  Eigen::MatrixXd Kd_fingertip_240;
  unsigned int delta_pos_update_frequency;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(fingertip_0_name));
    a->Visit(DRAKE_NVP(fingertip_120_name));
    a->Visit(DRAKE_NVP(fingertip_240_name));
    a->Visit(DRAKE_NVP(min_fingertips_delta_position));
    a->Visit(DRAKE_NVP(max_fingertips_delta_position));
    a->Visit(DRAKE_NVP(KpFingertip0));
    a->Visit(DRAKE_NVP(KdFingertip0));
    a->Visit(DRAKE_NVP(KpFingertip120));
    a->Visit(DRAKE_NVP(KdFingertip120));
    a->Visit(DRAKE_NVP(KpFingertip240));
    a->Visit(DRAKE_NVP(KdFingertip240));
    a->Visit(DRAKE_NVP(delta_pos_update_frequency));

    Kp_fingertip_0 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KpFingertip0.data(), 3, 3);
    Kd_fingertip_0 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KdFingertip0.data(), 3, 3);
    Kp_fingertip_120 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KpFingertip120.data(), 3, 3);
    Kd_fingertip_120 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KdFingertip120.data(), 3, 3);
    Kp_fingertip_240 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KpFingertip240.data(), 3, 3);
    Kd_fingertip_240 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KdFingertip240.data(), 3, 3);
  }
};
