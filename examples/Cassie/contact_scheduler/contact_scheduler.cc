#include "examples/Cassie/contact_scheduler/contact_scheduler.h"

#include <iostream>

using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::string;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::State;
using drake::systems::UnrestrictedUpdateEvent;

namespace dairlib {

using systems::ImpactInfoVector;
using systems::OutputVector;

ContactScheduler::ContactScheduler(
    const drake::multibody::MultibodyPlant<double> &plant,
    std::set<RUNNING_FSM_STATE> &impact_states, double near_impact_threshold,
    double tau, BLEND_FUNC blend_func)
    : near_impact_threshold_(near_impact_threshold),
      tau_(tau),
      blend_func_(blend_func),
      impact_states_(impact_states) {
  // Declare system ports
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();
  fsm_port_ = this->DeclareVectorOutputPort("fsm", BasicVector<double>(1),
                                            &ContactScheduler::CalcFiniteState)
      .get_index();
  impact_info_port_ = this->DeclareVectorOutputPort(
          "near_impact", ImpactInfoVector<double>(0, 0, 0),
          &ContactScheduler::CalcNextImpactInfo)
      .get_index();
  clock_port_ = this->DeclareVectorOutputPort("clock", BasicVector<double>(1),
                                              &ContactScheduler::CalcClock)
      .get_index();
  contact_scheduler_port_ =
      this->DeclareVectorOutputPort(
              "contact_scheduler (pelvis_t0, pelvis_tf, left_t0, left_tf, "
              "right_t0, right_t0",
              BasicVector<double>(6), &ContactScheduler::CalcContactScheduler)
          .get_index();

  // Declare discrete states and update
  stored_robot_state_index_ =
      DeclareDiscreteState(plant.num_positions() + plant.num_velocities());
  stored_fsm_state_index_ = DeclareDiscreteState(1);
  stored_transition_time_index_ = DeclareDiscreteState(1);
  nominal_state_durations_index_ = DeclareDiscreteState(2);
  std::vector<std::pair<double, RUNNING_FSM_STATE>>
      initial_state_transitions = {{0.1, LEFT_STANCE},
                                   {0.3, LEFT_FLIGHT},
                                   {0.4, RIGHT_STANCE},
                                   {0.6, RIGHT_FLIGHT}};
  upcoming_transitions_index_ = DeclareAbstractState(
      drake::Value<std::vector<std::pair<double, RUNNING_FSM_STATE>>>{
          initial_state_transitions});
  transition_times_index_ = DeclareAbstractState(
      drake::Value<std::vector<double>>{{0.1, 0.3, 0.4, 0.6}});

  DeclarePerStepUnrestrictedUpdateEvent(
      &ContactScheduler::UpdateTransitionTimes);
}

EventStatus ContactScheduler::UpdateTransitionTimes(
    const Context<double> &context, State<double> *state) const {
  const OutputVector<double> *robot_output =
      (OutputVector<double> *)
          this->EvalVectorInput(context, state_port_);
  double current_time = robot_output->get_timestamp();

  auto stored_fsm_state = (RUNNING_FSM_STATE) state->get_mutable_discrete_state(
      stored_fsm_state_index_)[0];
  double stored_transition_time =
      state->get_discrete_state(stored_transition_time_index_)[0];
  double nominal_stance_duration =
      state->get_discrete_state(nominal_state_durations_index_)[0];
  double nominal_flight_duration =
      state->get_discrete_state(nominal_state_durations_index_)[1];
  auto transition_times =
      state->get_mutable_abstract_state<std::vector<double>>(
          transition_times_index_);
  std::vector<std::pair<double, RUNNING_FSM_STATE>> &upcoming_transitions =
      state->get_mutable_abstract_state<
          std::vector<std::pair<double, RUNNING_FSM_STATE>>>(
          upcoming_transitions_index_);

  auto active_state = stored_fsm_state;
  double transition_time = upcoming_transitions.at(1).first;
  RUNNING_FSM_STATE transition_state = upcoming_transitions.at(1).second;
  if (current_time > transition_time) {
    active_state = transition_state;
  }

  if (active_state != stored_fsm_state) {
    state->get_mutable_discrete_state(stored_robot_state_index_)
        .SetFromVector(robot_output->GetState());
    state->get_mutable_discrete_state(stored_transition_time_index_)[0] =
        robot_output->get_timestamp();

    double g =
        drake::multibody::UniformGravityFieldElement<double>::kDefaultStrength;

    // Compute relative to stance foot
    double pelvis_z = robot_output->GetState()[6];
    double pelvis_zdot = robot_output->GetState()[23 + 5];

    if (active_state == LEFT_STANCE || active_state == RIGHT_STANCE) {
      // TODO(yangwill): calculate end of stance duration
      double next_transition_time = 0.3;
      state->get_mutable_discrete_state(nominal_state_durations_index_)[0] =
          next_transition_time - stored_transition_time;
      if (active_state == LEFT_STANCE) {
        transition_times[LEFT_FLIGHT] = next_transition_time;
        transition_times[RIGHT_STANCE] =
            next_transition_time + nominal_flight_duration;
        transition_times[RIGHT_FLIGHT] = next_transition_time +
            nominal_flight_duration +
            nominal_stance_duration;
        upcoming_transitions = {{transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT],
                                 RIGHT_FLIGHT}};
      } else {
        transition_times[LEFT_FLIGHT] = next_transition_time;
        transition_times[RIGHT_STANCE] =
            next_transition_time + nominal_flight_duration;
        transition_times[RIGHT_FLIGHT] = next_transition_time +
            nominal_flight_duration +
            nominal_stance_duration;
        upcoming_transitions = {{transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT}};
      }
    } else {
      // TODO calculate end of flight duration
      double next_transition_time =
          stored_transition_time +
              1 / g *
                  (pelvis_zdot + sqrt(pelvis_zdot * pelvis_zdot -
                      2 * g * (pelvis_z - rest_length_)));
      if (active_state == LEFT_FLIGHT) {
        transition_times[RIGHT_STANCE] = next_transition_time;
        transition_times[RIGHT_FLIGHT] =
            next_transition_time + nominal_stance_duration;
        transition_times[LEFT_STANCE] = next_transition_time +
            nominal_stance_duration +
            nominal_flight_duration;
        upcoming_transitions = {{transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE}};
      } else {
        transition_times[LEFT_STANCE] = next_transition_time;
        transition_times[LEFT_FLIGHT] =
            next_transition_time + nominal_stance_duration;
        transition_times[RIGHT_STANCE] = next_transition_time +
            nominal_stance_duration +
            nominal_flight_duration;
        upcoming_transitions = {{transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE}};
      }
    }
  }

  stored_fsm_state << active_state;
}

void ContactScheduler::CalcFiniteState(const Context<double> &context,
                                       BasicVector<double> *fsm_state) const {
  // Assign fsm_state
  fsm_state->SetFromVector(
      context.get_discrete_state(stored_fsm_state_index_).value());
}

void ContactScheduler::CalcNextImpactInfo(
    const Context<double> &context,
    ImpactInfoVector<double> *near_impact) const {
  // Read in lcm message time
  const OutputVector<double> *robot_output =
      (OutputVector<double> *)
          this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());
  const std::vector<std::pair<double, RUNNING_FSM_STATE>>
      &upcoming_transitions =
      context.get_abstract_state<
          std::vector<std::pair<double, RUNNING_FSM_STATE>>>(
          upcoming_transitions_index_);
  // Assign the blending function ptr
  auto alpha_func = blend_func_ == SIGMOID ? &blend_sigmoid : &blend_exp;

  near_impact->set_timestamp(current_time);
  //  near_impact->SetCurrentContactMode(0);
  //  near_impact->SetAlpha(0);

  // Get current finite state
  double transition_time = upcoming_transitions.at(1).first;
  RUNNING_FSM_STATE transition_state = upcoming_transitions.at(1).second;
  double blend_window = blend_func_ == SIGMOID ? 1.5 * near_impact_threshold_
                                               : near_impact_threshold_;
  // Check if the upcoming state has an impact
  if (impact_states_.count(transition_state) != 0) {
    // Check if we're close to an impact event
    if (abs(current_time - transition_time) < blend_window) {
      // Apply the corresponding blending function
      if (current_time < transition_time) {
        near_impact->SetAlpha(alpha_func(current_time - transition_time, tau_,
                                         near_impact_threshold_));
      } else {
        near_impact->SetAlpha(alpha_func(transition_time - current_time, tau_,
                                         near_impact_threshold_));
      }
      near_impact->SetCurrentContactMode(transition_state);
    }
  }
}

void ContactScheduler::CalcClock(const Context<double> &context,
                                 BasicVector<double> *clock) const {
  // Read in lcm message time
  const OutputVector<double> *robot_output =
      (OutputVector<double> *)
          this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  clock->get_mutable_value()(0) = current_time;
}

void ContactScheduler::CalcContactScheduler(
    const Context<double> &context, BasicVector<double> *contact_timing) const {
  // Read in lcm message time
  RUNNING_FSM_STATE current_state_ =
      (RUNNING_FSM_STATE) context.get_discrete_state(stored_fsm_state_index_)[0];
  auto transition_times =
      context.get_abstract_state<std::vector<double>>(transition_times_index_);
  if (current_state_ == LEFT_STANCE || current_state_ == RIGHT_STANCE) {
    contact_timing->get_mutable_value()(0) = transition_times[LEFT_STANCE];
    contact_timing->get_mutable_value()(1) = transition_times[LEFT_FLIGHT];
  } else {
    contact_timing->get_mutable_value()(0) = transition_times[RIGHT_STANCE];
    contact_timing->get_mutable_value()(1) = transition_times[RIGHT_FLIGHT];
  }
  contact_timing->get_mutable_value()(2) = transition_times[LEFT_FLIGHT];
  contact_timing->get_mutable_value()(3) = transition_times[LEFT_STANCE];
  contact_timing->get_mutable_value()(4) = transition_times[RIGHT_FLIGHT];
  contact_timing->get_mutable_value()(5) = transition_times[RIGHT_STANCE];
}

}  // namespace dairlib
