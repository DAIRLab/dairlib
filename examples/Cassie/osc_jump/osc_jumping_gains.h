#include "drake/common/yaml/yaml_read_archive.h"
#include "systems/controllers/osc/osc_gains.h"

using Eigen::MatrixXd;

struct OSCJumpingGains : OSCGains {
  // costs
  double crouch_x_offset;
  double land_x_offset;
  // center of mass tracking
  std::vector<double> CoMW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  // pelvis orientation tracking
  std::vector<double> PelvisRotW;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  // flight foot tracking
  std::vector<double> FlightFootW;
  std::vector<double> FlightFootKp;
  std::vector<double> FlightFootKd;
  // Swing toe tracking
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  // Hip yaw tracking
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double min_pelvis_acc;
  double max_pelvis_acc;
  double landing_delay;
  bool relative_feet;

  MatrixXd W_com;
  MatrixXd K_p_com;
  MatrixXd K_d_com;
  MatrixXd W_pelvis;
  MatrixXd K_p_pelvis;
  MatrixXd K_d_pelvis;
  MatrixXd W_flight_foot;
  MatrixXd K_p_flight_foot;
  MatrixXd K_d_flight_foot;
  MatrixXd W_hip_yaw;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(crouch_x_offset));
    a->Visit(DRAKE_NVP(land_x_offset));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisRotW));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(FlightFootW));
    a->Visit(DRAKE_NVP(FlightFootKp));
    a->Visit(DRAKE_NVP(FlightFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    a->Visit(DRAKE_NVP(min_pelvis_acc));
    a->Visit(DRAKE_NVP(max_pelvis_acc));
    a->Visit(DRAKE_NVP(landing_delay));
    a->Visit(DRAKE_NVP(relative_feet));

    W_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMW.data(), 3, 3);
    K_p_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMKp.data(), 3, 3);
    K_d_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMKd.data(), 3, 3);
    W_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotW.data(), 3, 3);
    K_p_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotKp.data(), 3, 3);
    K_d_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotKd.data(), 3, 3);
    W_flight_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FlightFootW.data(), 3, 3);
    K_p_flight_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FlightFootKp.data(), 3, 3);
    K_d_flight_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FlightFootKd.data(), 3, 3);
    W_hip_yaw = w_hip_yaw * MatrixXd::Identity(1, 1);
    K_p_hip_yaw = hip_yaw_kp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = hip_yaw_kd * MatrixXd::Identity(1, 1);
  }
};