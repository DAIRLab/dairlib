#include "systems/controllers/controller_failure_signal.h"

#include <limits>

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

ControllerFailureSignal::ControllerFailureSignal(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::vector<int> fsm_states_of_interest, int prev_fsm_state,
    bool set_current_time_until_first_state_switch)
    : fsm_states_of_interest_(fsm_states_of_interest),
      set_current_time_until_first_state_switch_(
          set_current_time_until_first_state_switch) {
  this->set_name("fsm_event_time");

  // Input/Output Setup
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  robot_output_port_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  start_time_port_ =
      this->DeclareVectorOutputPort(
          "t_start", BasicVector<double>(1),
          &ControllerFailureSignal::AssignStartTimeOfCurrentState)
          .get_index();
  if (!fsm_states_of_interest.empty()) {
    start_time_of_interest_port_ =
        this->DeclareVectorOutputPort(
            "t_start_state_of_interest", BasicVector<double>(1),
            &ControllerFailureSignal::AssignStartTimeOfStateOfInterest)
            .get_index();
  }

  // Per-step update to record the previous state and the previous event time
  DeclarePerStepDiscreteUpdateEvent(
      &ControllerFailureSignal::DiscreteVariableUpdate);
  // The start time of the current fsm state
  prev_time_idx_ = this->DeclareDiscreteState(1);
  // The start time of the most recent fsm state that we are interested
  prev_time_of_state_of_interest_idx_ = this->DeclareDiscreteState(1);

}

}  // namespace systems
}  // namespace dairlib
