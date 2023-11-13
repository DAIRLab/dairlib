#pragma once
#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_mpc_debug.hpp>
#include <dairlib/lcmt_mpc_solution.hpp>

#include "alip_utils.h"
#include "alip_mpfc_system.h"
#include "systems/filters/s2s_kalman_filter.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {

using alip_utils::PointOnFramed;

class AlipOneStepFootstepController : public drake::systems::LeafSystem<double> {
 public:

  // TODO (@Brian-Acosta) : Move stance durations to gains struct
  AlipOneStepFootstepController(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      std::vector<int> left_right_stance_fsm_states,
      std::vector<int> post_left_right_fsm_states,
      std::vector<double> left_right_stance_durations,
      double double_stance_duration,
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

 private:

  // System callbacks
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyFsmOutput(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* fsm) const;

  void CopyPrevImpactTimeOutput(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* t) const;

  void CopyNextFootstepOutput(const drake::systems::Context<double>& context,
                              drake::systems::BasicVector<double>* p_W) const;

  void CalcFootStepAndStanceFootHeight(
      const Eigen::Vector3d& stance_foot_pos_yaw_frame,
      const Eigen::Vector3d& com_pos_yaw_frame,
      const Eigen::Vector3d& L_yaw_frame,
      const Eigen::Vector2d& vdes_xy,
      const double curr_time,
      const double end_time_of_this_interval,
      int fsm_state,
      Eigen::Vector2d* x_fs) const;

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

  // drake output ports
  drake::systems::OutputPortIndex fsm_output_port_;
  drake::systems::OutputPortIndex next_impact_time_output_port_;
  drake::systems::OutputPortIndex prev_impact_time_output_port_;
  drake::systems::OutputPortIndex footstep_target_output_port_;

  // controller states
  drake::systems::DiscreteStateIndex fsm_state_idx_;
  drake::systems::DiscreteStateIndex next_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex prev_impact_time_state_idx_;
  drake::systems::DiscreteStateIndex footstep_target_state_idx_;

  // Multibody Plant Parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<int, const alip_utils::PointOnFramed> stance_foot_map_;
  int nq_;
  int nv_;
  int nu_;
  const double m_;

  // finite state machine management
  std::vector<int> left_right_stance_fsm_states_;
  std::vector<int> post_left_right_fsm_states_;
  double double_stance_duration_;
  double single_stance_duration_;

  // gains
  AlipMINLPGains gains_;
};
}