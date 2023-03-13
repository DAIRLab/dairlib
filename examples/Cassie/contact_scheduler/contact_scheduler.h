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

enum RunningFsmState { kLeftStance, kRightStance, kLeftFlight, kRightFlight };

/**
 * Variable timing finite state machine that predicts the liftoff and touchdown
 * timing assuming that the leg length is undeflected at touchdown and liftoff.
 * This system is also responsible for outputting the upcoming contact mode as
 * well as the blending function that is necessary for impact-invariant control
 *
 *
 *
 * @system
 * name: SLIPContactScheduler
 * input_ports:
 * - state_port: robot_state
 * output_ports:
 * - fsm_port: current contact mode (RunningFsmState)
 * - clock_port: current clock (phase variable)
 * - impact_info_port: upcoming contact/impact event (ImpactInfoVector)
 * - contact_scheduler_port: upcoming start and end times for pelvis and foot (BasicVector)
 * trajectories
 * - debug_port: lcm output for predicted contact mode switches
 * (lcmt_contact_timing)
 */
class SLIPContactScheduler : public drake::systems::LeafSystem<double> {
 public:
  SLIPContactScheduler(const drake::multibody::MultibodyPlant<double>& plant,
                       drake::systems::Context<double>* plant_context,
                       std::set<RunningFsmState> impact_states,
                       double near_impact_threshold = 0, double tau = 0.0025,
                       BLEND_FUNC blend_func = kSigmoid);

  void SetSLIPParams(double rest_length) { rest_length_ = rest_length; }
  void SetNominalStepTimings(double stance_duration, double flight_duration) {
    stance_duration_ = stance_duration;
    flight_duration_ = flight_duration;
  }
  void SetMaxStepTimingVariance(double stance_variance,
                                double flight_variance) {
    stance_variance_ = stance_variance;
    flight_variance_ = flight_variance;
  }
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
  const std::set<RunningFsmState> impact_states_;
  double near_impact_threshold_;
  double tau_;
  const BLEND_FUNC blend_func_;

  drake::systems::DiscreteStateIndex stored_fsm_state_index_;
  drake::systems::DiscreteStateIndex stored_robot_state_index_;
  drake::systems::DiscreteStateIndex stored_transition_time_index_;

  // estimates of state durations for stance and flight in seconds
  drake::systems::DiscreteStateIndex nominal_state_durations_index_;
  drake::systems::DiscreteStateIndex transition_times_index_;
  drake::systems::DiscreteStateIndex ground_height_index_;

  drake::systems::AbstractStateIndex upcoming_transitions_index_;

  /// SLIP parameters
  double rest_length_ = 0.0;
  double stance_duration_ = 0.0;
  double flight_duration_ = 0.0;
  double stance_variance_ = 0.0;
  double flight_variance_ = 0.0;
};

}  // namespace dairlib
