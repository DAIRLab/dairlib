#pragma once
#include "systems/controllers/footstep_planning/alip_minlp_footstep_controller.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"

using Eigen::MatrixXd;
using Eigen::Matrix4d;

struct AlipMINLPGainsImport {
  double t_commit;
  double t_min;
  double t_max;
  double u_max;
  double h_des;
  double stance_width;
  double ss_time;
  double ds_time;
  int nmodes;
  int knots_per_mode;
  std::vector<double> qf;
  std::vector<double> q;
  std::vector<double> r;
  Eigen::Matrix4d Qf;
  Eigen::Matrix4d Q;
  Eigen::MatrixXd R;

  dairlib::systems::controllers::AlipMINLPGains gains;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(t_commit));
    a->Visit(DRAKE_NVP(t_min));
    a->Visit(DRAKE_NVP(t_max));
    a->Visit(DRAKE_NVP(u_max));
    a->Visit(DRAKE_NVP(h_des));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(ss_time));
    a->Visit(DRAKE_NVP(ds_time));
    a->Visit(DRAKE_NVP(nmodes));
    a->Visit(DRAKE_NVP(knots_per_mode));
    a->Visit(DRAKE_NVP(qf));
    a->Visit(DRAKE_NVP(q));
    a->Visit(DRAKE_NVP(r));

    Qf = Eigen::Map<
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(this->qf.data());
    Q = Eigen::Map<
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(this->q.data());
    R = Eigen::Map<
        Eigen::Matrix<double, 1, 1, Eigen::RowMajor>>(this->r.data());

    this->gains = dairlib::systems::controllers::AlipMINLPGains {
        this->t_commit,
        this->t_min,
        this->t_max,
        this->u_max,
        this->h_des,
        this->stance_width,
        this->nmodes,
        this->knots_per_mode,
        this->Q,
        this->R
    };
  }
};