#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/blending_utils.h"
#include "dairlib/lcmt_contact_timing.hpp"
#include "systems/framework/impact_info_vector.h"
#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

enum RUNNING_FSM_STATE { LEFT_STANCE, RIGHT_STANCE, LEFT_FLIGHT, RIGHT_FLIGHT };

class ContactScheduler : public drake::systems::LeafSystem<double> {
 public:
  ContactScheduler(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* plant_context,
                   std::set<RUNNING_FSM_STATE>& impact_states,
                   double near_impact_threshold = 0, double tau = 0.0025,
                   BLEND_FUNC blend_func = SIGMOID);

  void SetSLIPParams(double rest_length) { rest_length_; }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() const {
    return this->get_output_port(fsm_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_clock() const {
    return this->get_output_port(clock_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_impact_info()
      const {
    return this->get_output_port(impact_info_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_contact_scheduler()
      const {
    return this->get_output_port(contact_scheduler_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_debug_info() const {
    return this->get_output_port(debug_port_);
  }

 private:
  drake::systems::EventStatus UpdateTransitionTimes(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;
  void CalcNextImpactInfo(const drake::systems::Context<double>& context,
                          systems::ImpactInfoVector<double>* near_impact) const;
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;
  void CalcClock(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* clock) const;
  void CalcContactScheduler(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* contact_timing) const;
  void OutputDebuggingInfo(const drake::systems::Context<double>& context,
                           dairlib::lcmt_contact_timing* debug_info) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::OutputPortIndex fsm_port_;
  drake::systems::OutputPortIndex clock_port_;
  drake::systems::OutputPortIndex impact_info_port_;
  drake::systems::OutputPortIndex contact_scheduler_port_;
  drake::systems::OutputPortIndex debug_port_;

  // Dynamics calculations
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* plant_context_;

  // For impact-invariant calculations
  const std::set<RUNNING_FSM_STATE> impact_states_;
  double near_impact_threshold_;
  double tau_;
  const BLEND_FUNC blend_func_;

  /// contains pairs (start of fsm, fsm_state)
  /// the order of the vector goes: last transition, next upcoming three
  /// transitions
  //  mutable std::vector<std::pair<double, RUNNING_FSM_STATE>>
  //  upcoming_transitions_; // sorted by upcoming time mutable
  //  std::vector<double> transition_times_; // fixed order by RUNNING_FSM_STATE

  int initial_state_ = 0;

  drake::systems::DiscreteStateIndex stored_fsm_state_index_;
  drake::systems::DiscreteStateIndex stored_robot_state_index_;
  drake::systems::DiscreteStateIndex stored_transition_time_index_;
  // estimates of state durations for stance and flight in seconds
  drake::systems::DiscreteStateIndex nominal_state_durations_index_;
  drake::systems::DiscreteStateIndex transition_times_index_;
  drake::systems::DiscreteStateIndex ground_height_index_;

  drake::systems::AbstractStateIndex upcoming_transitions_index_;

  /// SLIP parameters
  double rest_length_;
};

}  // namespace dairlib
