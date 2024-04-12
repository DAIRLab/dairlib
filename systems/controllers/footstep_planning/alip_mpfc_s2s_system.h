#pragma once
#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_alip_s2s_mpfc_debug.hpp>
#include <dairlib/lcmt_alip_mpc_output.hpp>

#include "alip_utils.h"
#include "alip_s2s_mpfc.h"
#include "geometry/convex_polygon_set.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {

using alip_utils::PointOnFramed;

class Alips2sMPFCSystem : public drake::systems::LeafSystem<double> {
 public:

  // TODO (@Brian-Acosta) : Move stance durations to gains struct
  Alips2sMPFCSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      std::vector<int> left_right_stance_fsm_states,
      std::vector<int> post_left_right_fsm_states,
      std::vector<PointOnFramed> left_right_foot,
      const alip_s2s_mpfc_params& mpfc_params);

  void MakeDrivenByStandaloneSimulator(double update_period) {
    DeclareInitializationUnrestrictedUpdateEvent(
        &Alips2sMPFCSystem::UnrestrictedUpdate);
    DeclarePeriodicUnrestrictedUpdateEvent(
        update_period, 0, &Alips2sMPFCSystem::UnrestrictedUpdate);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footholds() const {
    return this->get_input_port(foothold_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_elevation() const {
    return this->get_input_port(elevation_map_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_output() const {
    return this->get_output_port(mpc_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_mpc_debug() const {
    return this->get_output_port(mpc_debug_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_output_port_);
  }

 private:

  // System callbacks
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyMpcOutput(const drake::systems::Context<double>& context,
                     lcmt_alip_mpc_output* traj) const;
  void CopyAnkleTorque(const drake::systems::Context<double>& context,
                       lcmt_saved_traj* traj) const;
  void CopyFsmOutput(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* fsm) const;
  int GetFsmForOutput(const drake::systems::Context<double>& context) const;
  double GetPrevImpactTimeForOutput(
      const drake::systems::Context<double>& context) const;

  void CopyMpcDebugToLcm(const drake::systems::Context<double>& context,
                         lcmt_alip_s2s_mpfc_debug* mpc_debug) const;


  // FSM helper functions
  int curr_fsm(int fsm_idx) const {
    return left_right_stance_fsm_states_.at(fsm_idx);
  }
  int next_fsm(int fsm_idx) const {
    int next= fsm_idx + 1;
    if (next >= left_right_stance_fsm_states_.size()) {
      return curr_fsm(0);
    }
    return curr_fsm(next);
  }

  // drake input ports
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex vdes_input_port_;
  drake::systems::InputPortIndex foothold_input_port_;
  drake::systems::InputPortIndex elevation_map_port_;

  // drake output ports
  drake::systems::OutputPortIndex mpc_output_port_;
  drake::systems::OutputPortIndex mpc_debug_output_port_;
  drake::systems::OutputPortIndex fsm_output_port_;

  // controller states
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex prev_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex initial_conditions_state_idx_;

  // abstract states
  drake::systems::AbstractStateIndex mpc_solution_idx_;
  drake::systems::AbstractStateIndex footholds_idx_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<int, const alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;

  // mpc object
  mutable AlipS2SMPFC trajopt_;

  // finite state machine management
  std::vector<int> left_right_stance_fsm_states_;
  std::vector<int> post_left_right_fsm_states_;
  double double_stance_duration_;
  double single_stance_duration_;
};
}