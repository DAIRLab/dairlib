#include "examples/Cassie/contact_scheduler/contact_scheduler.h"

#include <iostream>
#include <utility>

#include <drake/math/saturate.h>

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

ContactScheduler::ContactScheduler(const MultibodyPlant<double>& plant,
                                   Context<double>* plant_context,
                                   std::set<RUNNING_FSM_STATE> impact_states,
                                   double near_impact_threshold, double tau,
                                   BLEND_FUNC blend_func)
    : plant_(plant),
      plant_context_(plant_context),
      impact_states_(std::move(impact_states)),
      near_impact_threshold_(near_impact_threshold),
      tau_(tau),
      blend_func_(blend_func) {
  this->set_name("ContactScheduler");
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
  stored_fsm_state_index_ = DeclareDiscreteState(3 * VectorXd::Ones(1));
  stored_robot_state_index_ =
      DeclareDiscreteState(plant.num_positions() + plant.num_velocities());
  stored_transition_time_index_ = DeclareDiscreteState(1);
  VectorXd nominal_state_durations = VectorXd::Zero(2);
  nominal_state_durations << 0.25, 0.1;
  nominal_state_durations_index_ =
      DeclareDiscreteState(nominal_state_durations);
  std::vector<std::pair<double, RUNNING_FSM_STATE>> initial_state_transitions =
      {{-0.1, RIGHT_FLIGHT},
       {0.00, LEFT_STANCE},
       {0.25, LEFT_FLIGHT},
       {0.35, RIGHT_STANCE}};
  //  transition_times_index_ = DeclareAbstractState(
  //      drake::Value<std::vector<double>>{{0.1, 0.3, 0.4, 0.6}});
  VectorXd initial_transition_times = VectorXd::Zero(4);
  initial_transition_times << -0.1, 0.0, 0.25, 0.35;
  transition_times_index_ = DeclareDiscreteState(initial_transition_times);
  ground_height_index_ = DeclareDiscreteState(VectorXd::Zero(1));

  upcoming_transitions_index_ = DeclareAbstractState(
      drake::Value<std::vector<std::pair<double, RUNNING_FSM_STATE>>>{
          initial_state_transitions});
  debug_port_ = this->DeclareAbstractOutputPort(
                        "status", &ContactScheduler::OutputDebuggingInfo)
                    .get_index();

  DeclarePerStepUnrestrictedUpdateEvent(
      &ContactScheduler::UpdateTransitionTimes);
}

EventStatus ContactScheduler::UpdateTransitionTimes(
    const Context<double>& context, State<double>* state) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double current_time = robot_output->get_timestamp();

  auto stored_fsm_state = (RUNNING_FSM_STATE)state->get_mutable_discrete_state(
      stored_fsm_state_index_)[0];
  double stored_transition_time =
      state->get_discrete_state(stored_transition_time_index_)[0];
  double nominal_stance_duration =
      state->get_discrete_state(nominal_state_durations_index_)[0];
  double nominal_flight_duration =
      state->get_discrete_state(nominal_state_durations_index_)[1];
  auto transition_times =
      state->get_mutable_discrete_state(transition_times_index_)
          .get_mutable_value();
  std::vector<std::pair<double, RUNNING_FSM_STATE>>& upcoming_transitions =
      state->get_mutable_abstract_state<
          std::vector<std::pair<double, RUNNING_FSM_STATE>>>(
          upcoming_transitions_index_);

  auto active_state = stored_fsm_state;
  double transition_time = upcoming_transitions.at(3).first;
  RUNNING_FSM_STATE transition_state = upcoming_transitions.at(3).second;
  if (current_time > transition_time) {
    //    std::cout << "transitioning to: " << transition_state << std::endl;
    //    std::cout << "at: " << transition_time << std::endl;
    active_state = transition_state;
  }
  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, plant_context_);
  if (active_state == LEFT_STANCE) {
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
  } else if (active_state == RIGHT_STANCE) {
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
    double pelvis_z = robot_output->GetPositionAtIndex(6) -
                      state->get_discrete_state(ground_height_index_)[0];
    double pelvis_zdot = robot_output->GetVelocityAtIndex(5);

    if (active_state == LEFT_STANCE || active_state == RIGHT_STANCE) {
      // Store the ground height of the stance foot

      // TODO(yangwill): calculate end of stance duration
      double next_transition_time = stored_transition_time + 0.25;
      state->get_mutable_discrete_state(nominal_state_durations_index_)[0] =
          next_transition_time - stored_transition_time;
      if (active_state == LEFT_STANCE) {
        transition_times[LEFT_FLIGHT] = next_transition_time;
        upcoming_transitions = {{transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT}};
      } else {
        transition_times[RIGHT_FLIGHT] = next_transition_time;
        upcoming_transitions = {{transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT}};
      }
    } else {
      double time_to_touchdown = 0.1;
      if (pelvis_zdot * pelvis_zdot - 2 * g * (rest_length_ - pelvis_z) > 0) {
        time_to_touchdown =
            (pelvis_zdot + sqrt(pelvis_zdot * pelvis_zdot -
                                2 * g * (rest_length_ - pelvis_z))) /
            g;
      }
      double next_transition_time =
          stored_transition_time +
          drake::math::saturate(time_to_touchdown, 0.05, 0.15);
      state->get_mutable_discrete_state(nominal_state_durations_index_)[1] =
          next_transition_time - stored_transition_time;
      if (active_state == LEFT_FLIGHT) {
        transition_times[RIGHT_STANCE] = next_transition_time;
        upcoming_transitions = {{transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE},
                                {transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE}};
      } else {
        transition_times[LEFT_STANCE] = next_transition_time;
        upcoming_transitions = {{transition_times[LEFT_FLIGHT], LEFT_FLIGHT},
                                {transition_times[RIGHT_STANCE], RIGHT_STANCE},
                                {transition_times[RIGHT_FLIGHT], RIGHT_FLIGHT},
                                {transition_times[LEFT_STANCE], LEFT_STANCE}};
      }
    }
  }

  state->get_mutable_discrete_state(stored_fsm_state_index_).get_mutable_value()
      << active_state;
  return drake::systems::EventStatus::Succeeded();
}

void ContactScheduler::CalcFiniteState(const Context<double>& context,
                                       BasicVector<double>* fsm_state) const {
  // Assign fsm_state
  VectorXd current_finite_state =
      context.get_discrete_state(stored_fsm_state_index_).CopyToVector();
  fsm_state->get_mutable_value() = current_finite_state;
}

void ContactScheduler::CalcNextImpactInfo(
    const Context<double>& context,
    ImpactInfoVector<double>* near_impact) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());
  const std::vector<std::pair<double, RUNNING_FSM_STATE>>&
      upcoming_transitions = context.get_abstract_state<
          std::vector<std::pair<double, RUNNING_FSM_STATE>>>(
          upcoming_transitions_index_);
  // Assign the blending function ptr
  auto alpha_func = blend_func_ == SIGMOID ? &blend_sigmoid : &blend_exp;

  near_impact->set_timestamp(current_time);
  //  near_impact->SetCurrentContactMode(0);
  near_impact->SetAlpha(0);

  // Get current finite state
  double transition_time = upcoming_transitions.at(3).first;
  RUNNING_FSM_STATE transition_state = upcoming_transitions.at(3).second;
  double previous_transition_time = upcoming_transitions.at(2).first;
  RUNNING_FSM_STATE current_state = upcoming_transitions.at(2).second;
  double blend_window = blend_func_ == SIGMOID ? 1.5 * near_impact_threshold_
                                               : near_impact_threshold_;
  // Check if we're close to an impact event either upcoming or one that just
  // happened

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

void ContactScheduler::CalcClock(const Context<double>& context,
                                 BasicVector<double>* clock) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  clock->get_mutable_value()(0) = current_time;
}

void ContactScheduler::CalcContactScheduler(
    const Context<double>& context, BasicVector<double>* contact_timing) const {
  // Read in lcm message time
  RUNNING_FSM_STATE current_state_ =
      (RUNNING_FSM_STATE)context.get_discrete_state(stored_fsm_state_index_)[0];
  double nominal_stance_duration =
      context.get_discrete_state(nominal_state_durations_index_)[0];
  double nominal_flight_duration =
      context.get_discrete_state(nominal_state_durations_index_)[1];
  auto transition_times =
      context.get_discrete_state(transition_times_index_).value();
  if (current_state_ == LEFT_STANCE || current_state_ == RIGHT_STANCE) {
    contact_timing->get_mutable_value()(0) = transition_times[LEFT_STANCE];
    contact_timing->get_mutable_value()(1) = transition_times[LEFT_FLIGHT];
  } else {
    contact_timing->get_mutable_value()(0) = transition_times[RIGHT_STANCE];
    contact_timing->get_mutable_value()(1) = transition_times[RIGHT_FLIGHT];
  }
  contact_timing->get_mutable_value()(2) = transition_times[LEFT_FLIGHT];
  contact_timing->get_mutable_value()(3) = transition_times[LEFT_FLIGHT] +
                                           2 * nominal_flight_duration +
                                           nominal_stance_duration;
  contact_timing->get_mutable_value()(4) = transition_times[RIGHT_FLIGHT];
  contact_timing->get_mutable_value()(5) = transition_times[RIGHT_FLIGHT] +
                                           2 * nominal_flight_duration +
                                           nominal_stance_duration;
  //  std::cout << "contact_scheduler start: "
  //            << contact_timing->get_mutable_value()(2) << std::endl;
  //  std::cout << "contact_scheduler end: "
  //            << contact_timing->get_mutable_value()(3) << std::endl;
}

void ContactScheduler::OutputDebuggingInfo(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_contact_timing* debug_info) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto& upcoming_transitions = context.get_abstract_state<
      std::vector<std::pair<double, RUNNING_FSM_STATE>>>(
      upcoming_transitions_index_);
  auto stored_fsm_state =
      (RUNNING_FSM_STATE)context.get_discrete_state(stored_fsm_state_index_)[0];
  debug_info->utime = robot_output->get_timestamp() * 1e6;

  debug_info->n_transitions = 4;
  debug_info->active_fsm_state = stored_fsm_state;
  debug_info->transition_times.clear();
  debug_info->transition_times = std::vector<double>(4);
  debug_info->transition_states.clear();
  debug_info->transition_states = std::vector<int16_t>(4);
  debug_info->fsm_state_names = {"LEFT_STANCE (0)", "RIGHT_STANCE (1)",
                                 "LEFT_FLIGHT (2)", "RIGHT_FLIGHT (3)"};

  for (int i = 0; i < debug_info->n_transitions; ++i) {
    debug_info->transition_times.at(i) = upcoming_transitions[i].first;
    debug_info->transition_states.at(i) = upcoming_transitions[i].second;
  }
}

}  // namespace dairlib
