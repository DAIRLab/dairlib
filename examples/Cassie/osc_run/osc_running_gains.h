#include "systems/controllers/osc/osc_gains.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCRunningGains : OSCGains {
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double w_hip_pitch;
  double hip_pitch_kp;
  double hip_pitch_kd;
  double w_hip_roll;
  double hip_roll_kp;
  double hip_roll_kd;
  // swing foot tracking
  std::vector<double> SwingFootW;
  std::vector<double> SwingFootKp;
  std::vector<double> SwingFootKd;
  // pelvis tracking
  std::vector<double> PelvisW;
  std::vector<double> PelvisKp;
  std::vector<double> PelvisKd;
  // pelvis orientation tracking
  std::vector<double> FootstepKp;
  std::vector<double> FootstepKd;

  MatrixXd K_p_footstep;
  MatrixXd K_d_footstep;
  MatrixXd W_pelvis;
  MatrixXd K_p_pelvis;
  MatrixXd K_d_pelvis;
  MatrixXd W_swing_foot;
  MatrixXd K_p_swing_foot;
  MatrixXd K_d_swing_foot;
  MatrixXd W_swing_toe;
  MatrixXd K_p_swing_toe;
  MatrixXd K_d_swing_toe;
  MatrixXd W_hip_yaw;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;
  MatrixXd W_hip_pitch;
  MatrixXd K_p_hip_pitch;
  MatrixXd K_d_hip_pitch;
  MatrixXd W_hip_roll;
  MatrixXd K_p_hip_roll;
  MatrixXd K_d_hip_roll;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(mu));

    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(PelvisKp));
    a->Visit(DRAKE_NVP(PelvisKd));
    a->Visit(DRAKE_NVP(FootstepKp));
    a->Visit(DRAKE_NVP(FootstepKd));
    a->Visit(DRAKE_NVP(SwingFootW));
    a->Visit(DRAKE_NVP(SwingFootKp));
    a->Visit(DRAKE_NVP(SwingFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    a->Visit(DRAKE_NVP(w_hip_pitch));
    a->Visit(DRAKE_NVP(hip_pitch_kp));
    a->Visit(DRAKE_NVP(hip_pitch_kd));
    a->Visit(DRAKE_NVP(w_hip_roll));
    a->Visit(DRAKE_NVP(hip_roll_kp));
    a->Visit(DRAKE_NVP(hip_roll_kd));

    W_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootW.data(), 3, 3);
    K_p_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKp.data(), 3, 3);
    K_d_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKd.data(), 3, 3);
    W_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisW.data(), 3, 3);
    K_p_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisKp.data(), 3, 3);
    K_d_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisKd.data(), 3, 3);
    K_p_footstep = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FootstepKp.data(), 2, 2);
    K_d_footstep = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FootstepKd.data(), 2, 2);

    W_swing_toe = this->w_swing_toe * MatrixXd::Identity(1, 1);
    K_p_swing_toe = this->swing_toe_kp * MatrixXd::Identity(1, 1);
    K_d_swing_toe = this->swing_toe_kd * MatrixXd::Identity(1, 1);
    W_hip_yaw = this->w_hip_yaw * MatrixXd::Identity(1, 1);
    K_p_hip_yaw = this->hip_yaw_kp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = this->hip_yaw_kd * MatrixXd::Identity(1, 1);
    W_hip_pitch = this->w_hip_pitch * MatrixXd::Identity(1, 1);
    K_p_hip_pitch = this->hip_pitch_kp * MatrixXd::Identity(1, 1);
    K_d_hip_pitch = this->hip_pitch_kd * MatrixXd::Identity(1, 1);
    W_hip_roll = this->w_hip_roll * MatrixXd::Identity(1, 1);
    K_p_hip_roll = this->hip_roll_kp * MatrixXd::Identity(1, 1);
    K_d_hip_roll = this->hip_roll_kd * MatrixXd::Identity(1, 1);
  }
};