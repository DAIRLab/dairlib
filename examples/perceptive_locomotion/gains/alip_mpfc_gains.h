#pragma once
#include "systems/controllers/footstep_planning/alip_mpfc_system.h"
#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using dairlib::systems::controllers::alip_utils::ResetDiscretization;

struct AlipMpfcGainsImport {
  double retraction_dist;
  double t_commit;
  double t_min;
  double t_max;
  double u_max;
  double h_des;
  double stance_width;
  double r_foostep_const;
  double ss_time;
  double ds_time;
  double xlim;
  int nmodes;
  int knots_per_mode;
  int solver_threads;
  std::vector<double> qf;
  std::vector<double> q;
  std::vector<double> r;
  std::vector<double> w_footstep_reg;
  int pelvis_vel_butter_order;
  std::vector<double> pelvis_vel_butter_wc;
  std::string reset_discretization_method;
  bool filter_alip_state;
  std::vector<double> qfilt;
  std::vector<double> rfilt;
  Eigen::Matrix4d Qf;
  Eigen::Matrix4d Q;
  Eigen::MatrixXd R;
  Eigen::Vector4d Qfilt_diagonal;
  Eigen::Vector4d Rfilt_diagonal;
  Eigen::Matrix3d W_footstep_reg;

  dairlib::systems::controllers::AlipMINLPGains gains;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(t_commit));
    a->Visit(DRAKE_NVP(t_min));
    a->Visit(DRAKE_NVP(t_max));
    a->Visit(DRAKE_NVP(u_max));
    a->Visit(DRAKE_NVP(h_des));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(r_foostep_const));
    a->Visit(DRAKE_NVP(ss_time));
    a->Visit(DRAKE_NVP(ds_time));
    a->Visit(DRAKE_NVP(nmodes));
    a->Visit(DRAKE_NVP(knots_per_mode));
    a->Visit(DRAKE_NVP(qf));
    a->Visit(DRAKE_NVP(q));
    a->Visit(DRAKE_NVP(r));
    a->Visit(DRAKE_NVP(filter_alip_state));
    a->Visit(DRAKE_NVP(qfilt));
    a->Visit(DRAKE_NVP(rfilt));
    a->Visit(DRAKE_NVP(reset_discretization_method));
    a->Visit(DRAKE_NVP(pelvis_vel_butter_order));
    a->Visit(DRAKE_NVP(pelvis_vel_butter_wc));
    a->Visit(DRAKE_NVP(solver_threads));
    a->Visit(DRAKE_NVP(w_footstep_reg));
    a->Visit(DRAKE_NVP(xlim));
    a->Visit(DRAKE_NVP(retraction_dist));

    Qf = Eigen::Map<
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(this->qf.data());
    Q = Eigen::Map<
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(this->q.data());
    R = Eigen::Map<
        Eigen::Matrix<double, 1, 1, Eigen::RowMajor>>(this->r.data());
    W_footstep_reg = Eigen::Map<
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(this->w_footstep_reg.data());
    Qfilt_diagonal = Eigen::Vector4d::Map(this->qfilt.data());
    Rfilt_diagonal = Eigen::Vector4d::Map(this->rfilt.data());

    DRAKE_DEMAND(reset_discretization_method == "ZOH" ||
                 reset_discretization_method == "FOH" ||
                 reset_discretization_method == "SPLIT");
    ResetDiscretization reset_disc = ResetDiscretization::kZOH;
    if (reset_discretization_method == "FOH") {
      reset_disc = ResetDiscretization::kFOH;
    }
    if (reset_discretization_method == "SPLIT") {
      reset_disc = ResetDiscretization::kSPLIT;
    }

    this->gains = dairlib::systems::controllers::AlipMINLPGains {
        this->t_commit,
        this->t_min,
        this->t_max,
        this->u_max,
        this->h_des,
        this->stance_width,
        this->r_foostep_const,
        this->xlim,
        this->nmodes,
        this->knots_per_mode,
        reset_disc,
        this->filter_alip_state,
        this->Q,
        this->Qf,
        this->W_footstep_reg,
        this->R
    };
  }

  void SetFilterData(double m, double H) {
    Eigen::Matrix4d A = dairlib::systems::controllers::alip_utils::CalcA(H, m);
    Eigen::MatrixXd B = -Eigen::MatrixXd::Identity(4,2);
    MatrixXd C = MatrixXd::Identity(4,4);
    MatrixXd G = MatrixXd::Identity(4,4);
    Eigen::Matrix4d Qfilt = this->Qfilt_diagonal.asDiagonal();
    Eigen::Matrix4d Rfilt = this->Rfilt_diagonal.asDiagonal();
    this->gains.filter_data = {{A, B, C, Qfilt, Rfilt}, G};
  }
};