#include "systems/controllers/fsm_event_time.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

namespace dairlib {
namespace systems {

FiniteStateMachineEventTime::FiniteStateMachineEventTime(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::vector<int> fsm_states_of_interest)
    : fsm_states_of_interest_(fsm_states_of_interest) {
  this->set_name("fsm_event_time");

  // Input/Output Setup
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  robot_output_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  start_time_port_ =
      this->DeclareVectorOutputPort(
              BasicVector<double>(1),
              &FiniteStateMachineEventTime::AssignStartTimeOfCurrentState)
          .get_index();
  if (!fsm_states_of_interest.empty()) {
    start_time_of_interest_port_ =
        this->DeclareVectorOutputPort(
                BasicVector<double>(1),
                &FiniteStateMachineEventTime::AssignStartTimeOfStateOfInterest)
            .get_index();
  }
}

void FiniteStateMachineEventTime::AssignStartTimeOfCurrentState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* current_state_start_time) const {
  // Read in current finite state machine state
  double fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()(0);

  // when entering a new state which is in fsm_states_of_interest
  if (fsm_state != prev_fsm_state0_) {
    prev_fsm_state0_ = fsm_state;

    // Record time
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, robot_output_port_);
    prev_time_ = static_cast<double>(robot_output->get_timestamp());
          cout << "prev_time_ = " << prev_time_ << endl;

  }

  // Assign
  current_state_start_time->get_mutable_value() << prev_time_;
}

void FiniteStateMachineEventTime::AssignStartTimeOfStateOfInterest(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* state_start_time) const {
  // Read in current finite state machine state
  double fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()(0);

  cout << "-----------------------------------------\n";
  cout << "fsm_state = " << fsm_state << endl;
  cout << "prev_fsm_state1_ = " << prev_fsm_state1_ << endl;
  cout << "context.get_time() = " << context.get_time() << endl;

  // when entering a new state which is in fsm_states_of_interest
  if (fsm_state != prev_fsm_state1_) {
    prev_fsm_state1_ = fsm_state;

    // Check if the current state is of our interest
    auto it = find(fsm_states_of_interest_.begin(),
                   fsm_states_of_interest_.end(), int(fsm_state));
    bool is_state_of_interest = it != fsm_states_of_interest_.end();
    if (is_state_of_interest) {
      // Record time
      const OutputVector<double>* robot_output =
          (OutputVector<double>*)this->EvalVectorInput(context, robot_output_port_);
      prev_time_of_state_of_interest_ = static_cast<double>(robot_output->get_timestamp());
//      cout << "prev_time_of_state_of_interest_ = " << prev_time_of_state_of_interest_ << endl;
      cout << "Change to a new state!!!!!!!!!!!!!!\n";
    }
  }
  cout << "prev_time_of_state_of_interest_ = " << prev_time_of_state_of_interest_ << endl;

  // Assign
  state_start_time->get_mutable_value() << prev_time_of_state_of_interest_;
}

}  // namespace systems
}  // namespace dairlib
