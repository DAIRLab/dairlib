#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

struct GaitParams {
  double t_ss;
  double t_ds;
  double mpc_dt;
  double mpc_N;
  Eigen::VectorXd standing_pose_q;
  Eigen::VectorXd standing_pose_lambda;
  Eigen::VectorXd standing_pose_u;
  std::string left_foot_body_name;
  std::string right_foot_body_name;
  std::vector<std::string> left_foot_contacts;
  std::vector<std::string> right_foot_contacts;
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

  bool is_double_stance() const {
    return state == kPostLeftDouble || state == kPostRightDouble;
  }

  fsm_state state = kPostRightDouble;
  double prev_switch_time{};
  double next_switch_time{};
};

class WalkingReferenceSystem : public drake::systems::LeafSystem<double> {
 public:

  WalkingReferenceSystem(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *plant_context,
      const GaitParams& params);

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_state()
  const {
    return get_input_port(input_port_state_);
  }

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_vdes()
  const {
    return get_input_port(input_port_vdes_);
  }

  void AddSwingFootTrajCostToMPC(IDMPC* mpc) const;

 private:

  drake::systems::EventStatus
  UnrestrictedUpdate(const drake::systems::Context<double>& context,
                drake::systems::State<double>* state) const;

  void CalcReference(
      double t, const Eigen::VectorXd& q, const fsm_info& fsm, MPCReference* mpc_reference) const;

  fsm_info CalcFSM(double t, const fsm_info& curr_fsm) const;

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  GaitParams params_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_vdes_;

  drake::systems::AbstractStateIndex reference_state_idx_;
  drake::systems::AbstractStateIndex fsm_info_idx_;
};

}