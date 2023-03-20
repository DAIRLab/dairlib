#include "examples/Cassie/contact_scheduler/contact_scheduler.h"

#include <iostream>
#include <utility>

#include "common/eigen_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using drake::multibody::Frame;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::State;
using drake::systems::UnrestrictedUpdateEvent;

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using systems::ImpactInfoVector;
using systems::OutputVector;

SLIPContactScheduler::SLIPContactScheduler(const MultibodyPlant<double>& plant,
                                           Context<double>* plant_context,
                                           std::set<RunningFsmState> impact_states,
                                           double near_impact_threshold, double tau,
                                           BLEND_FUNC blend_func)
    : plant_(plant),
      plant_context_(plant_context),
      impact_states_(std::move(impact_states)),
      near_impact_threshold_(near_impact_threshold),
      tau_(tau),
      blend_func_(blend_func) {
  DRAKE_DEMAND(tau_ > 0);
  DRAKE_DEMAND(near_impact_threshold_ >= 0);
  this->set_name("SLIPContactScheduler");
  // Declare system ports
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorOutputPort("fsm", 1,
                                            &SLIPContactScheduler::CalcFiniteState)
                  .get_index();
  impact_info_port_ = this->DeclareVectorOutputPort(
                              "near_impact", ImpactInfoVector<double>(0, 0, 0),
                              &SLIPContactScheduler::CalcNextImpactInfo)
                          .get_index();
  clock_port_ = this->DeclareVectorOutputPort("clock", 1,
                                              &SLIPContactScheduler::CalcClock)
                    .get_index();
  contact_scheduler_port_ =
      this->DeclareVectorOutputPort(
              "contact_scheduler (pelvis_t0, pelvis_tf, left_t0, left_tf, "
              "right_t0, right_t0",
              6, &SLIPContactScheduler::CalcContactScheduler)
          .get_index();
  debug_port_ = this->DeclareAbstractOutputPort(
          "status", &SLIPContactScheduler::OutputDebuggingInfo)
      .get_index();

  // Declare discrete states and update
  stored_fsm_state_index_ = DeclareDiscreteState(3 * VectorXd::Ones(1));
  stored_robot_state_index_ =
      DeclareDiscreteState(plant.num_positions() + plant.num_velocities());
  stored_transition_time_index_ = DeclareDiscreteState(1);
  VectorXd nominal_state_durations = VectorXd::Zero(2);
  nominal_state_durations << 0.25, 0.1;
  nominal_state_durations_index_ =
      DeclareDiscreteState(nominal_state_durations);
  std::vector<std::pair<double, RunningFsmState>> initial_state_transitions =
      {{-0.1, kRightFlight},
       {0.00, kLeftStance},
       {0.25, kLeftFlight},
       {0.35, kRightStance}};
  VectorXd initial_transition_times = VectorXd::Zero(4);
  initial_transition_times << -0.1, 0.0, 0.25, 0.35;
  transition_times_index_ = DeclareDiscreteState(initial_transition_times);
  ground_height_index_ = DeclareDiscreteState(VectorXd::Zero(1));

  upcoming_transitions_index_ = DeclareAbstractState(
      drake::Value<std::vector<std::pair<double, RunningFsmState>>>{
          initial_state_transitions});

  DeclarePerStepUnrestrictedUpdateEvent(
      &SLIPContactScheduler::UpdateTransitionTimes);
}

EventStatus SLIPContactScheduler::UpdateTransitionTimes(
    const Context<double>& context, State<double>* state) const {
  DRAKE_DEMAND(rest_length_ > 0);
  DRAKE_DEMAND(stance_duration_ > 0);
  DRAKE_DEMAND(flight_duration_ > 0);

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double current_time = robot_output->get_timestamp();

  auto stored_fsm_state = (RunningFsmState)state->get_mutable_discrete_state(
      stored_fsm_state_index_)[0];
  auto transition_times =
      state->get_mutable_discrete_state(transition_times_index_)
          .get_mutable_value();
  auto& upcoming_transitions =
      state->get_mutable_abstract_state<
          std::vector<std::pair<double, RunningFsmState>>>(
          upcoming_transitions_index_);

  auto active_state = stored_fsm_state;
  double transition_time = upcoming_transitions.at(3).first;
  RunningFsmState transition_state = upcoming_transitions.at(3).second;
  if (current_time > transition_time) {
    active_state = transition_state;
  }
  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, plant_context_);
  if (active_state == kLeftStance) {
    auto toe_front = LeftToeFront(plant_);
    auto toe_rear = LeftToeRear(plant_);
    Vector3d toe_front_pos;
    Vector3d toe_rear_pos;
    plant_.CalcPointsPositions(*plant_context_, toe_front.second,
                               toe_front.first, plant_.world_frame(),
                               &toe_front_pos);
    plant_.CalcPointsPositions(*plant_context_, toe_rear.second, toe_rear.first,
                               plant_.world_frame(), &toe_rear_pos);
    VectorXd height =
        0.5 * (toe_front_pos[2] + toe_rear_pos[2]) * VectorXd::Ones(1);
    state->get_mutable_discrete_state(ground_height_index_)
        .SetFromVector(height);
  } else if (active_state == kRightStance) {
    auto toe_front = RightToeFront(plant_);
    auto toe_rear = RightToeRear(plant_);
    Vector3d toe_front_pos;
    Vector3d toe_rear_pos;
    plant_.CalcPointsPositions(*plant_context_, toe_front.second,
                               toe_front.first, plant_.world_frame(),
                               &toe_front_pos);
    plant_.CalcPointsPositions(*plant_context_, toe_rear.second, toe_rear.first,
                               plant_.world_frame(), &toe_rear_pos);
    VectorXd height =
        0.5 * (toe_front_pos[2] + toe_rear_pos[2]) * VectorXd::Ones(1);
    state->get_mutable_discrete_state(ground_height_index_)
        .SetFromVector(height);
  }

  if (active_state != stored_fsm_state) {
    state->get_mutable_discrete_state(stored_robot_state_index_)
        .SetFromVector(robot_output->GetState());
    state->get_mutable_discrete_state(stored_transition_time_index_)[0] =
        robot_output->get_timestamp();
    double stored_transition_time =
        state->get_discrete_state(stored_transition_time_index_)[0];
    double g =
        drake::multibody::UniformGravityFieldElement<double>::kDefaultStrength;

    // Compute relative to stance foot
    VectorXd pelvis_pos = Vector3d::Zero();
    plant_.CalcPointsPositions(
        *plant_context_, plant_.GetBodyByName("pelvis").body_frame(),
        Vector3d::Zero(), plant_.world_frame(), &pelvis_pos);
    double pelvis_z =
        pelvis_pos[2] -
        state->get_discrete_state(ground_height_index_).value()[0];
    double pelvis_zdot = robot_output->GetVelocityAtIndex(5);

    if (active_state == kLeftStance || active_state == kRightStance) {
      // This is a very crude approximation
      double stance_scale = (pelvis_z) / (rest_length_);
      stance_scale =
          std::clamp(stance_scale, 1 - stance_variance_, 1 + stance_variance_);
      double next_transition_time =
          stored_transition_time + stance_scale * stance_duration_;
      state->get_mutable_discrete_state(nominal_state_durations_index_)[0] =
          next_transition_time - stored_transition_time;
      if (active_state == kLeftStance) {
        transition_times[kLeftFlight] = next_transition_time;
        upcoming_transitions = {{transition_times[kRightStance], kRightStance},
                                {transition_times[kRightFlight], kRightFlight},
                                {transition_times[kLeftStance], kLeftStance},
                                {transition_times[kLeftFlight], kLeftFlight}};
      } else {
        transition_times[kRightFlight] = next_transition_time;
        upcoming_transitions = {{transition_times[kLeftStance], kLeftStance},
                                {transition_times[kLeftFlight], kLeftFlight},
                                {transition_times[kRightStance], kRightStance},
                                {transition_times[kRightFlight], kRightFlight}};
      }
    } else {
      // set default to minimum touchdown time in case pelvis is below rest
      // length
      double time_to_touchdown = (1 - flight_variance_) * flight_duration_;
      if (pelvis_zdot * pelvis_zdot - 2 * g * (rest_length_ - pelvis_z) > 0) {
        time_to_touchdown =
            (pelvis_zdot + sqrt(pelvis_zdot * pelvis_zdot -
                                2 * g * (rest_length_ - pelvis_z))) /
            g;
      }
      double time_to_touchdown_saturated = std::clamp(
          time_to_touchdown, (1 - flight_variance_) * flight_duration_,
          (1 + flight_variance_) * flight_duration_);
      double next_transition_time =
          stored_transition_time + time_to_touchdown_saturated;
      state->get_mutable_discrete_state(nominal_state_durations_index_)[1] =
          next_transition_time - stored_transition_time;
      if (active_state == kLeftFlight) {
        transition_times[kRightStance] = next_transition_time;
        upcoming_transitions = {{transition_times[kRightFlight], kRightFlight},
                                {transition_times[kLeftStance], kLeftStance},
                                {transition_times[kLeftFlight], kLeftFlight},
                                {transition_times[kRightStance], kRightStance}};
      } else {
        transition_times[kLeftStance] = next_transition_time;
        upcoming_transitions = {{transition_times[kLeftFlight], kLeftFlight},
                                {transition_times[kRightStance], kRightStance},
                                {transition_times[kRightFlight], kRightFlight},
                                {transition_times[kLeftStance], kLeftStance}};
      }
    }
  }

  state->get_mutable_discrete_state(stored_fsm_state_index_).get_mutable_value()
      << active_state;
  return drake::systems::EventStatus::Succeeded();
}

void SLIPContactScheduler::CalcFiniteState(const Context<double>& context,
                                           BasicVector<double>* fsm_state) const {
  // Assign fsm_state
  VectorXd current_finite_state =
      context.get_discrete_state(stored_fsm_state_index_).CopyToVector();
  fsm_state->get_mutable_value() = current_finite_state;
}

void SLIPContactScheduler::CalcNextImpactInfo(
    const Context<double>& context,
    ImpactInfoVector<double>* near_impact) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());
  const auto& upcoming_transitions = context.get_abstract_state<
      std::vector<std::pair<double, RunningFsmState>>>(
      upcoming_transitions_index_);
  // Assign the blending function ptr
  auto alpha_func = blend_func_ == kSigmoid ? &blend_sigmoid : &blend_exp;

  near_impact->set_timestamp(current_time);
  near_impact->SetAlpha(0);

  // Get current finite state
  double transition_time = upcoming_transitions.at(3).first;
  RunningFsmState transition_state = upcoming_transitions.at(3).second;
  double previous_transition_time = upcoming_transitions.at(2).first;
  RunningFsmState current_state = upcoming_transitions.at(2).second;
  double blend_window = blend_func_ == kSigmoid ? 1.5 * near_impact_threshold_
                                                : near_impact_threshold_;
  // Check if the upcoming state has an impact
  if (impact_states_.count(transition_state) != 0) {
    if (abs(current_time - transition_time) < blend_window) {
      // Apply the corresponding blending function
      if (current_time <= transition_time) {
        near_impact->SetAlpha(alpha_func(current_time - transition_time, tau_,
                                         near_impact_threshold_));
      }
      near_impact->SetCurrentContactMode(transition_state);
    }
  }
  // Check if current state that we just transitioned to has an impact
  if (impact_states_.count(current_state) != 0) {
    if (abs(current_time - previous_transition_time) < blend_window) {
      // Apply the corresponding blending function
      if (current_time >= previous_transition_time) {
        near_impact->SetAlpha(
            alpha_func(previous_transition_time - current_time, tau_,
                       near_impact_threshold_));
      }
      near_impact->SetCurrentContactMode(current_state);
    }
  }
}

void SLIPContactScheduler::CalcClock(const Context<double>& context,
                                     BasicVector<double>* clock) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  clock->get_mutable_value()(0) = current_time;
}

void SLIPContactScheduler::CalcContactScheduler(
    const Context<double>& context, BasicVector<double>* contact_timing) const {
  // Read in lcm message time
  auto current_state_ =
      (RunningFsmState)context.get_discrete_state(stored_fsm_state_index_)[0];
  double nominal_stance_duration =
      context.get_discrete_state(nominal_state_durations_index_)[0];
  double nominal_flight_duration =
      context.get_discrete_state(nominal_state_durations_index_)[1];
  auto transition_times =
      context.get_discrete_state(transition_times_index_).value();
  /// Timing indices: order goes from start time to end time for when the
  /// tracking objective is active.
  /// PelvisTouchdown = 0
  /// PelvisLiftoff = 1
  /// LeftFootLiftoff = 2
  /// LeftFootTouchdown = 3
  /// RightFootLiftoff = 4
  /// RightFootTouchdown = 5
  if (current_state_ == kLeftStance || current_state_ == kRightFlight) {
    contact_timing->get_mutable_value()(0) = transition_times[kLeftStance];
    contact_timing->get_mutable_value()(1) = transition_times[kLeftFlight];
  } else {
    contact_timing->get_mutable_value()(0) = transition_times[kRightStance];
    contact_timing->get_mutable_value()(1) = transition_times[kRightFlight];
  }
  contact_timing->get_mutable_value()(2) = transition_times[kLeftFlight];
  contact_timing->get_mutable_value()(3) = transition_times[kLeftFlight] +
                                           2 * nominal_flight_duration +
                                           nominal_stance_duration;
  contact_timing->get_mutable_value()(4) = transition_times[kRightFlight];
  contact_timing->get_mutable_value()(5) = transition_times[kRightFlight] +
                                           2 * nominal_flight_duration +
                                           nominal_stance_duration;
}

void SLIPContactScheduler::OutputDebuggingInfo(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_contact_timing* debug_info) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto& upcoming_transitions = context.get_abstract_state<
      std::vector<std::pair<double, RunningFsmState>>>(
      upcoming_transitions_index_);
  auto stored_fsm_state =
      (RunningFsmState)context.get_discrete_state(stored_fsm_state_index_)[0];
  debug_info->utime = robot_output->get_timestamp() * 1e6;

  debug_info->n_transitions = 4;
  debug_info->active_fsm_state = stored_fsm_state;
  debug_info->transition_times.clear();
  debug_info->transition_times = std::vector<double>(4);
  debug_info->transition_states.clear();
  debug_info->transition_states = std::vector<int16_t>(4);
  debug_info->fsm_state_names = {"kLeftStance (0)", "kRightStance (1)",
                                 "kLeftFlight (2)", "kRightFlight (3)"};

  for (int i = 0; i < debug_info->n_transitions; ++i) {
    debug_info->transition_times.at(i) = upcoming_transitions[i].first;
    debug_info->transition_states.at(i) = upcoming_transitions[i].second;
  }
}

}  // namespace dairlib
