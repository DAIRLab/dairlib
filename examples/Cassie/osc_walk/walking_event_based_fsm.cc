#include "examples/Cassie/osc_walk/walking_event_based_fsm.h"

#include <drake/lcmt_contact_results_for_viz.hpp>

using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
namespace examples {
namespace osc_walk {

WalkingEventFsm::WalkingEventFsm(const MultibodyPlant<double>& plant,
                                 const vector<double>& transition_times,
                                 bool contact_based, FSM_STATE init_state,
                                 bool print_fsm_info)
    : transition_times_(transition_times),
      contact_based_(contact_based),
      init_state_(init_state),
      print_fsm_info_(print_fsm_info) {
  DRAKE_ASSERT(transition_times_.size() == RIGHT_STANCE + 1);
  for (auto& t : transition_times_) {
    std::cout << "fsm time: " << t << std::endl;
  }
  state_port_ =
      this->DeclareVectorInputPort("x",OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  // Configure the contact info port for the particular simulator
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();
  this->DeclareVectorOutputPort("fsm", BasicVector<double>(1),
                                &WalkingEventFsm::CalcFiniteState);
  DeclarePerStepDiscreteUpdateEvent(&WalkingEventFsm::DiscreteVariableUpdate);

  BasicVector<double> init_prev_time = BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> init_fsm_state = BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> time_offset = BasicVector<double>(VectorXd::Zero(1));
  init_fsm_state.get_mutable_value()(0) = init_state_;

  prev_time_idx_ = this->DeclareDiscreteState(init_prev_time);
  fsm_idx_ = this->DeclareDiscreteState(init_fsm_state);
  time_offset_idx_ = this->DeclareDiscreteState(time_offset);
}

EventStatus WalkingEventFsm::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Get inputs to the leaf system
  const auto state_feedback =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto& contact_info =
      this->EvalInputValue<drake::lcmt_contact_results_for_viz>(context,
                                                                contact_port_);
  double timestamp = state_feedback->get_timestamp();

  // Get the discrete states
  auto fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();
  auto time_offset =
      discrete_state->get_mutable_vector(time_offset_idx_).get_mutable_value();

  // Simulator has restarted, reset FSM
  if (timestamp < prev_time(0)) {
    std::cout << "Simulator has restarted!" << std::endl;
    std::cout << "Initial state is : " << init_state_ << std::endl;
    fsm_state << init_state_;
    prev_time << timestamp;
    time_offset << 0.0;
  }

  int num_contacts = contact_info->num_point_pair_contacts;
  if (contact_based_ && num_contacts == 0) {
    fsm_state << init_state_;
    return EventStatus::Failed(this, "No feet are in contact with the ground!");
  }
  string contact_point;
  if (contact_based_ && num_contacts <= 2) {
    contact_point = contact_info->point_pair_contact_info[0].body2_name;
  }

  // To test delayed switching times, there is an "intermediate" state
  // between each state change when the guard condition is first triggered
  // The fsm state will change transition_delay_ seconds after the guard
  // condition was first triggered.
  // This supports both contact-based and time-based guard conditions
  if (fsm_state(0) == LEFT_STANCE) {
    if (contact_based_
            ? num_contacts >= 3
            : timestamp > (transition_times_[LEFT_STANCE] + time_offset(0))) {
      SetNextFiniteState(fsm_state, timestamp);
    }
  } else if (fsm_state(0) == RIGHT_STANCE) {
    if (contact_based_
            ? num_contacts >= 3
            : timestamp > (transition_times_[RIGHT_STANCE] + time_offset(0))) {
      SetNextFiniteState(fsm_state, timestamp);
      time_offset << timestamp;
    }
  }
  prev_time(0) = timestamp;

  return EventStatus::Succeeded();
}  // namespace examples

void WalkingEventFsm::CalcFiniteState(const Context<double>& context,
                                      BasicVector<double>* fsm_state) const {
  fsm_state->get_mutable_value() =
      context.get_discrete_state().get_vector(fsm_idx_).get_value();
}

void WalkingEventFsm::SetNextFiniteState(Eigen::VectorBlock<VectorXd> fsm_state,
                                         double timestamp) const {
  if (fsm_state(0) == RIGHT_STANCE)
    fsm_state << LEFT_STANCE;
  else
    fsm_state(0) += 1;
  if (print_fsm_info_) {
    std::cout << "Current time: " << timestamp << "\n";
    std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << "\n";
  }
}

}  // namespace osc_walk
}  // namespace examples
}  // namespace dairlib
