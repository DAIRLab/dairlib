#pragma once
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::systems::controllers::id_mpc {

struct GaitParams {
  int mpc_N;
  int footstep_horizon;
  double t_ss;
  double t_ds;
  double mpc_dt;
  double step_height;
  double stance_width;
  double pelvis_height;
  Eigen::VectorXd standing_pose_q;
  Eigen::VectorXd standing_pose_lambda;
  Eigen::VectorXd standing_pose_u;
  Eigen::Matrix3d foot_pos_W;
  std::string left_foot_body_name;
  std::string right_foot_body_name;
  std::string floating_base_name;
  std::vector<std::string> left_foot_contacts;
  std::vector<std::string> right_foot_contacts;
  std::vector<int> left_leg_actuator_idxs;
  std::vector<int> right_leg_actuator_idxs;
  std::vector<int> left_leg_holonomic_constraint_idxs;
  std::vector<int> right_leg_holonomic_constraint_idxs;
  Eigen::Vector3d foot_midpoint;
};

/*!
 * Struct to support loading the subset of parameters which are user-set
 */
struct GaitParamsLoader {
  int footstep_horizon;
  double t_ss;
  double t_ds;
  double step_height;
  double stance_width;
  double pelvis_height;
  std::vector<double> foot_pos_w;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(footstep_horizon));
    a->Visit(DRAKE_NVP(t_ss));
    a->Visit(DRAKE_NVP(t_ds));
    a->Visit(DRAKE_NVP(step_height));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(pelvis_height));
    a->Visit(DRAKE_NVP(foot_pos_w));
  }

  static GaitParams LoadUserGaitParamsFromYaml(const std::string& filename) {
    const auto archive = drake::yaml::LoadYamlFile<GaitParamsLoader>(filename);
    DRAKE_DEMAND(archive.foot_pos_w.size() == 3);

    GaitParams out;
    out.footstep_horizon = archive.footstep_horizon;
    out.t_ss = archive.t_ss;
    out.t_ds = archive.t_ds;
    out.step_height = archive.step_height;
    out.stance_width = archive.stance_width;
    out.pelvis_height = archive.pelvis_height;
    out.foot_pos_W = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 3; ++i) {
      out.foot_pos_W(i,i) = archive.foot_pos_w.at(i);
    }
    return out;
  }

};

struct fsm_info {
  enum fsm_state {
    kLeft = 0,
    kPostLeftDouble = 1,
    kRight = 2,
    kPostRightDouble = 3
  };

  static fsm_state next_fsm(fsm_state curr) {
    if (curr == kLeft) return kPostLeftDouble;
    if (curr == kPostLeftDouble) return kRight;
    if (curr == kRight) return kPostRightDouble;
    return kLeft;
  }

  friend std::ostream &operator<<(std::ostream &out, const fsm_state &s) {
    switch (s) {
      case kLeft: out << "kLeft";
        break;
      case kPostLeftDouble: out << "kPostLeftDouble";
        break;
      case kRight: out << "kRight";
        break;
      case kPostRightDouble: out << "kPostRightDouble";
        break;
      default: out << s;
    }
    return out;
  }

  bool is_double_stance() const {
    return state == kPostLeftDouble || state == kPostRightDouble;
  }

  static bool is_double_stance(fsm_state s) {
    return s == kPostLeftDouble || s == kPostRightDouble;
  }

  fsm_state state = kPostRightDouble;
  double prev_switch_time{};
  double next_switch_time{};
};

}