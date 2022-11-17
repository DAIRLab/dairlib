#include "contact_scheduler.h"

#include <iostream>

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using std::string;

namespace dairlib {
namespace systems {

ContactScheduler::ContactScheduler(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::unordered_map<int, int>& contact_state_to_fsm_state_map)
    : contact_state_to_fsm_state_map_(contact_state_to_fsm_state_map) {
  // Input/Output Setup
  state_input_port_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
  contact_force_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lambda", drake::Value<PiecewisePolynomial<double>>(pp))
          .get_index();
  fsm_output_port_ =
      this->DeclareVectorOutputPort("fsm", BasicVector<double>(1),
                                    &ContactScheduler::CalcFiniteState)
          .get_index();
}

void ContactScheduler::CalcFiniteState(const Context<double>& context,
                                       BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());
  const auto& contact_force_trajectory =
      this->EvalInputValue<drake::trajectories::PiecewisePolynomial<double>>(
          context, contact_force_trajectory_input_port_);
  VectorXd desired_contact = contact_force_trajectory->value(current_sim_time);

  /// shift the active contacts to form contact state
  const int contact_state = ((desired_contact[2] > kEpislonForce) * 1 << 0) +
                            ((desired_contact[5] > kEpislonForce) * 1 << 1) +
                            ((desired_contact[8] > kEpislonForce) * 1 << 2) +
                            ((desired_contact[11] > kEpislonForce) * 1 << 3);

  // Assign fsm_state
  fsm_state->get_mutable_value()[0] =
      contact_state_to_fsm_state_map_.at(contact_state);
}

}  // namespace systems
}  // namespace dairlib
