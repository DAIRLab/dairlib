#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerControllerParams : OSCGains {
  std::string trifinger_model;
  std::string cube_model;
  std::string fingertip_0_name;
  std::string fingertip_120_name;
  std::string fingertip_240_name;

  Eigen::Vector3d min_fingertips_delta_position;
  Eigen::Vector3d max_fingertips_delta_position;

  std::vector<double> home_position;

  std::vector<double> Fingertip0W;
  std::vector<double> Fingertip0Kp;
  std::vector<double> Fingertip0Kd;
  std::vector<double> Fingertip120W;
  std::vector<double> Fingertip120Kp;
  std::vector<double> Fingertip120Kd;
  std::vector<double> Fingertip240W;
  std::vector<double> Fingertip240Kp;
  std::vector<double> Fingertip240Kd;

  Eigen::MatrixXd W_fingertip_0;
  Eigen::MatrixXd Kp_fingertip_0;
  Eigen::MatrixXd Kd_fingertip_0;
  Eigen::MatrixXd W_fingertip_120;
  Eigen::MatrixXd Kp_fingertip_120;
  Eigen::MatrixXd Kd_fingertip_120;
  Eigen::MatrixXd W_fingertip_240;
  Eigen::MatrixXd Kp_fingertip_240;
  Eigen::MatrixXd Kd_fingertip_240;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);

    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(cube_model));
    a->Visit(DRAKE_NVP(fingertip_0_name));
    a->Visit(DRAKE_NVP(fingertip_120_name));
    a->Visit(DRAKE_NVP(fingertip_240_name));
    a->Visit(DRAKE_NVP(min_fingertips_delta_position));
    a->Visit(DRAKE_NVP(max_fingertips_delta_position));
    a->Visit(DRAKE_NVP(Fingertip0W));
    a->Visit(DRAKE_NVP(Fingertip0Kp));
    a->Visit(DRAKE_NVP(Fingertip0Kd));
    a->Visit(DRAKE_NVP(Fingertip120W));
    a->Visit(DRAKE_NVP(Fingertip120Kp));
    a->Visit(DRAKE_NVP(Fingertip120Kd));
    a->Visit(DRAKE_NVP(Fingertip240W));
    a->Visit(DRAKE_NVP(Fingertip240Kp));
    a->Visit(DRAKE_NVP(Fingertip240Kd));
    a->Visit(DRAKE_NVP(home_position));

    W_fingertip_0 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip0W.data(), 3, 3);
    Kp_fingertip_0 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip0Kp.data(), 3, 3);
    Kd_fingertip_0 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip0Kd.data(), 3, 3);
    W_fingertip_120 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip120W.data(), 3, 3);
    Kp_fingertip_120 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip120Kp.data(), 3, 3);
    Kd_fingertip_120 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip120Kd.data(), 3, 3);
    W_fingertip_240 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip240W.data(), 3, 3);
    Kp_fingertip_240 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip240Kp.data(), 3, 3);
    Kd_fingertip_240 = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->Fingertip240Kd.data(), 3, 3);
  }
};