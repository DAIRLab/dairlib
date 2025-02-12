#pragma once
#include <iostream>
#include "Eigen/Dense"

namespace dairlib::systems::controllers::id_mpc {

struct GaitParams {
  double t_ss;
  double t_ds;
  double mpc_dt;
  int mpc_N;
  double stance_width;
  Eigen::VectorXd standing_pose_q;
  Eigen::VectorXd standing_pose_lambda;
  Eigen::VectorXd standing_pose_u;
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

  friend std::ostream &operator<<(std::ostream &out, const fsm_state &state) {
    switch (state) {
      case kLeft: out << "kLeft";
        break;
      case kPostLeftDouble: out << "kPostLeftDouble";
        break;
      case kRight: out << "kRight";
        break;
      case kPostRightDouble: out << "kPostRightDouble";
        break;
      default: out << state;
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