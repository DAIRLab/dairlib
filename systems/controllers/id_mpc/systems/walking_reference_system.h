#pragma once

#include "systems/controllers/id_mpc/id_mpc.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

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

class WalkingReferenceSystem : public drake::systems::LeafSystem<double> {
 public:

  WalkingReferenceSystem(
      const ConstrainedDynamicsInfo& dynamics,
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

  void AddSwingFootTrajCostToMPC(IDMPC* mpc, const Eigen::Matrix3d& Q) const;

 private:

  drake::systems::EventStatus
  UnrestrictedUpdate(const drake::systems::Context<double>& context,
                drake::systems::State<double>* state) const;

  fsm_info CalcFSM(double t, const fsm_info& curr_fsm) const;

  std::vector<fsm_info::fsm_state> CalcGaitTiming(
      double t, const fsm_info& fsm, MPCReference* mpc_reference) const;

  void SetContactsAtKnot(
      int i, fsm_info::fsm_state state, MPCReference* ref) const;

  // Note that these methods assume the context is up-to-date with the
  // robot's current state
  // TODO (@Brian-Acosta) include yaw rate in the reference
  // TODO (@Brian-Acosta) add height lookup for position
  drake::trajectories::PiecewisePolynomial<double> CalcPositionTraj(
      fsm_info::fsm_state fsm_state, const Eigen::Vector2d& vdes,
      const std::vector<double>& breaks) const;
  drake::trajectories::PiecewisePolynomial<double> CalcOrientationTraj() const;
  drake::trajectories::PiecewisePolynomial<double> CalcVelocityTraj(
      const Eigen::Vector2d& vdes) const;

  drake::trajectories::PiecewisePolynomial<double> CalcInputTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state> fsm_states) const;

  drake::trajectories::PiecewisePolynomial<double> CalcLambdaTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_states,
      const std::vector<std::vector<std::string>>& active_contacts) const;

  drake::trajectories::PiecewisePolynomial<double> CalcSwingFootTraj(
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_states,
      const std::vector<double> phases) const;

  std::vector<double> CalcSSPhaseVector(
      const fsm_info& fsm,
      const std::vector<double>& breaks,
      const std::vector<fsm_info::fsm_state>& fsm_vector) const;

  const ConstrainedDynamicsInfo& dynamics_;
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  GaitParams params_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_vdes_;

  drake::systems::AbstractStateIndex reference_state_idx_;
  drake::systems::AbstractStateIndex fsm_info_idx_;

  int ds_intervals_;
  int ss_intervals_;
};

}