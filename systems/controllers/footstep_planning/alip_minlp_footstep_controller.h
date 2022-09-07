#pragma once
#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_mpc_debug.hpp>

#include "alip_utils.h"
#include "alip_minlp.h"
#include "geometry/convex_foothold.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {
using alip_utils::PointOnFramed;

struct AlipMINLPGains {
  double t_commit;
  double t_min;
  double hdes;
  double stance_width;
  int nmodes;
  int knots_per_mode;
  Eigen::Matrix4d Q;
  Eigen::MatrixXd R;
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

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footholds() const {
    return this->get_input_port(foothold_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_next_impact_time()
  const {
    return this->get_output_port(next_impact_time_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_prev_impact_time()
  const {
    return this->get_output_port(prev_impact_time_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_footstep_target()
  const {
    return this->get_output_port(footstep_target_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com_traj() const {
    return this->get_output_port(com_traj_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_debug() const {
    return this->get_output_port(mpc_debug_output_port_);
  }

 private:

  // System callbacks
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyNextFootstepOutput(const drake::systems::Context<double>& context,
                              drake::systems::BasicVector<double>* p_W) const;
  void CopyCoMTrajOutput(const drake::systems::Context<double>& context,
                         lcmt_saved_traj* traj_msg) const;
  void CopyMpcSolutionToLcm(const drake::systems::Context<double>& context,
                            lcmt_mpc_debug* mpc_debug) const;


  // drake input ports
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex foothold_input_port_;

  // drake output ports
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex next_impact_time_output_port_;
  drake::systems::OutputPortIndex prev_impact_time_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;
  drake::systems::OutputPortIndex com_traj_output_port_;
  drake::systems::OutputPortIndex mpc_debug_output_port_;

  // controller states
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex prev_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex initial_conditions_state_idx_;
  // abstract states
  drake::systems::AbstractStateIndex alip_minlp_index_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<int, const alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;

  // Nominal FSM
  std::vector<int> left_right_stance_fsm_states_;
  std::map<int, double> stance_duration_map_;
  double double_stance_duration_;

  // gains
  AlipMINLPGains gains_;
};
}