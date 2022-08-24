#pragma once

#include "alip_utils.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/framework/output_vector.h"
#include "geometry/convex_foothold.h"

namespace dairlib::systems::controllers {

struct AlipS2SControllerParameters {
  // maybe filter the alip state
  bool filter_alip_state = false;
  Eigen::MatrixXd Q_filt;
  Eigen::MatrixXd R_filt;

  // footstep controller gains
  Eigen::MatrixXd W_xy;
  double nominal_foot_y;
  double step_period_update_cutoff_time;
  double z_com_des;
  double Kt;
};

class AlipS2SPlanner : public drake::systems::LeafSystem<double> {
 public:
  AlipS2SPlanner(const drake::multibody::MultibodyPlant<double>& plant,
                 drake::systems::Context<double>* context,
                 std::vector<int> left_right_stance_fsm_states,
                 std::vector<double> left_right_stance_durations,
                 std::vector<alip_utils::PointOnFramed> left_right_foot,
                 const AlipS2SControllerParameters& params);
 private:
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  Eigen::Vector2d CalcNominalAlipFootTarget(
      Eigen::Vector2d vdes, Eigen::Vector4d alip_state,
      double time_to_impact,
      drake::EigenPtr<Eigen::Vector2d> p_sw,
      drake::EigenPtr<Eigen::Vector2d> p_com) const;

  double CalcFootstepTimingOffset(const geometry::ConvexFoothold& foothold,
      const Eigen::Vector2d& p_sw, const Eigen::Vector2d& p_com) const;

  // drake input ports
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex foothold_input_port_;


  // drake output ports
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex next_impact_time_output_port_;
  drake::systems::OutputPortIndex time_since_impact_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;
  drake::systems::OutputPortIndex com_traj_output_port_;

  // controller state
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_idx_;
  drake::systems::DiscreteStateIndex time_since_impact_idx_;
  drake::systems::DiscreteStateIndex footstep_target_idx_;
  drake::systems::DiscreteStateIndex zdot_com_pre_impact_idx_;

  // abstract state and QP solution caching
  drake::systems::AbstractStateIndex s2s_filter_idx_;
  drake::systems::AbstractStateIndex planar_alip_mpc_idx_; // unused for now
  drake::systems::AbstractStateIndex z_com_mpc_idx_;       // unused for now
  drake::systems::AbstractStateIndex planar_alip_mpc_sol_idx_;
  drake::systems::AbstractStateIndex z_com_mpc_sol_idx_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  std::map<int, alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;
  double m_;

  // Nominal FSM
  std::vector<int> left_right_stance_fsm_states_;
  std::map<int, double> duration_map_;
  double double_stance_duration_;

  // "Gains" and other parameters
  const AlipS2SControllerParameters& params_;

};
}