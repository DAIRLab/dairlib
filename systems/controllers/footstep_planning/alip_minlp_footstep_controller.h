#pragma once
#include <dairlib/lcmt_saved_traj.hpp>

#include "alip_utils.h"
#include "alip_minlp.h"
#include "geometry/convex_foothold.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {
using alip_utils::PointOnFramed;

struct AlipMINLPGains {
  double T_min_until_touchdown;
  double hdes;
  int nmodes;
  int knots_per_mode;
  Eigen::Matrix4d Q;
};

class AlipMINLPFootstepController : public drake::systems::LeafSystem<double> {
 public:
  AlipMINLPFootstepController(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      std::vector<int> left_right_stance_fsm_states,
      std::vector<double> left_right_stance_durations,
      std::vector<PointOnFramed> left_right_foot,
      const AlipMINLPGains& gains);
 private:

  // System callbacks
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

//  void CopyFsmOutput(const drake::systems::Context<double>& context,
//                     drake::systems::BasicVector<double>* fsm);
//  void CopyTimeSinceSwitchOutput(const drake::systems::Context<double>& context,
//                                 drake::systems::BasicVector<double>* t);
//  void CopyTimeUntilSwitchOutput(const drake::systems::Context<double>& context,
//                                 drake::systems::BasicVector<double>* t);
  void CopyNextFootstepOutput(const drake::systems::Context<double>& context,
                              drake::systems::BasicVector<double>* p_B_FC);
  void CopyCoMTrajOutput(const drake::systems::BasicVector<double>& context,
                         lcmt_saved_traj* traj_msg);


  // drake input ports
  drake::systems::InputPortIndex robot_state_index_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex foothold_input_port_;

  // drake output ports
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex next_impact_time_output_port_;
  drake::systems::OutputPortIndex prev_impact_time_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;
  drake::systems::OutputPortIndex com_traj_output_port_;

  // controller states
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex prev_impact_time_state_idx_;

  // abstract states
  drake::systems::AbstractStateIndex alip_minlp_index_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<int, alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;

  // Nominal FSM
  std::vector<int> left_right_stance_fsm_states_;
  std::map<int, double> stance_duration_map_;
  double double_stance_duration_;

  // mathematical program
  drake::solvers::SnoptSolver solver_;

  // gains
  AlipMINLPGains gains_;
};
}