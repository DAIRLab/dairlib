

#include "joint_level_controller.h"

#include <drake/systems/framework/event_status.h>

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

JointLevelController::JointLevelController(
    const MultibodyPlant<double>& plant,
    const drake::trajectories::PiecewisePolynomial<double>& state_traj,
    const drake::trajectories::PiecewisePolynomial<double>& input_traj,
    const Eigen::MatrixXd& K, const Eigen::MatrixXd& map, int max_periods)
    : plant_(plant),
      state_traj_(state_traj),
      input_traj_(input_traj),
      K_(K),
      map_(map),
      max_periods_(max_periods),
      nq_(plant.num_positions()),
      nv_(plant.num_velocities()),
      nu_(plant.num_actuators()) {
  DRAKE_ASSERT(K_.rows() == nu_);
  DRAKE_ASSERT(K_.cols() == nq_ + nv_);
  input_port_info_index_ =
      this->DeclareVectorInputPort("x, u, t", OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  output_port_efforts_index_ =
      this->DeclareVectorOutputPort("u, t",
              TimestampedVector<double>(plant.num_actuators()),
              &JointLevelController::CalcControl)
          .get_index();
  time_shift_idx_ = this->DeclareDiscreteState(1);
  DeclarePerStepDiscreteUpdateEvent(
      &JointLevelController::DiscreteVariableUpdate);
}

EventStatus JointLevelController::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto robot_output = this->template EvalVectorInput<OutputVector>(
      context, input_port_info_index_);
  double timestamp = robot_output->get_timestamp();

  if (abs(timestamp - max_periods_ * state_traj_.end_time()) < 1e-3) {
    // TODO(yangwill) Calculate efforts to result in landing using fixed point solver
  }

  return EventStatus::Succeeded();
}

void JointLevelController::CalcControl(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* control) const {
  auto state = (OutputVector<double>*)this->EvalVectorInput(
      context, input_port_info_index_);
  double timestamp = state->get_timestamp();
//  double phase = std::min(max_periods_ * state_traj_.end_time(), timestamp);
  double phase = std::fmod(timestamp, state_traj_.end_time() - 0.2);
  VectorXd u_ff = input_traj_.value(phase);
  VectorXd u_fb = -K_ * (state->GetState() - map_ * state_traj_.value(phase));
  control->set_timestamp(timestamp);
  control->SetDataVector(u_ff + u_fb);
}

}  // namespace systems
}  // namespace dairlib
