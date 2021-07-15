#include "systems/dynamics_residual_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

using drake::systems::Context;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

DynamicResidualSystem::DynamicResidualSystem(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<multibody::KinematicEvaluatorSet<double>*>& evaluators,
    const double tau)
    : plant_(plant), evaluators_(evaluators), tau_(tau) {
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

  plant_context_ = plant.CreateDefaultContext();

  // Declare state
  residual_idx_ = this->DeclareDiscreteState(nq + nv);
  prev_state_idx_ = this->DeclareDiscreteState(nq + nv);
  prev_time_idx_ = this->DeclareDiscreteState(1);
  is_init_idx_ = this->DeclareDiscreteState(1);

  this->DeclareForcedDiscreteUpdateEvent(&DynamicResidualSystem::UpdateState);

  robot_output_port_ = this->DeclareVectorInputPort(OutputVector<double>(nq, nv, nu))
                    .get_index();

  mode_port_ =
      this->DeclareVectorInputPort(drake::systems::BasicVector<double>(1))
          .get_index();

  this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(nq + nv),
                                &DynamicResidualSystem::CopyOutput);
}

EventStatus DynamicResidualSystem::UpdateState(
    const Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  // Continuous dynamics: d/dt y = a * (f(x) - xddot - y)
  // Discretized version:
  //    y[k+1] = (1 - dt * a) * y + dt * a * f(x) + a * [x[k+1] - x[k]]

  const auto state_input =
      this->template EvalVectorInput<OutputVector>(context, robot_output_port_);

  double t = state_input->get_timestamp();
  double t_prev =
      context.get_discrete_state(prev_time_idx_).GetAtIndex(0);
  double is_init =
      context.get_discrete_state(is_init_idx_).GetAtIndex(0);
  double dt = t - t_prev;

  std::cout << "dt: " << dt << std::endl;
  const VectorXd x = state_input->GetState();
  const VectorXd u = state_input->GetEfforts();

  if (dt > 0 && is_init == 1) {
    multibody::setContext<double>(plant_, x, u, plant_context_.get());

    // const int mode = this->EvalVectorInput(context, mode_port_)->GetAtIndex(0);
    const int mode = 2; // TODO:"REMOVE"

    auto x_prev = context.get_discrete_state(prev_state_idx_).get_value();

    // Calculate dynamics
    auto xdot = evaluators_[mode]->CalcTimeDerivatives(*plant_context_);

    std::cout << "x" << std::endl << x << std::endl << std::endl;
    std::cout << "u" << std::endl << u << std::endl << std::endl;

    // Update residual
    auto residual_prev = context.get_discrete_state(residual_idx_).get_value();

    VectorXd error = dt * xdot;// - x + x_prev;

    Eigen::MatrixXd M(plant_.num_velocities(), plant_.num_velocities());
    plant_.CalcMassMatrix(*plant_context_, &M);
    error.tail(plant_.num_velocities()) =
        M * error.tail(plant_.num_velocities());

    auto residual = residual_prev +
                    (1 / tau_) * (error - dt * residual_prev);

    Eigen::MatrixXd tmp(x.rows(), 2);
    tmp.col(0) = xdot;
    tmp.col(1) = (x - x_prev) / dt;
    std::cout << tmp << std::endl << std::endl;

    discrete_state->get_mutable_vector(residual_idx_).SetFromVector(residual);
  }

  // Copy timestamp
  discrete_state->get_mutable_vector(prev_time_idx_).SetAtIndex(0, t);

  // Copy state
  discrete_state->get_mutable_vector(prev_state_idx_).SetFromVector(x);

  // Set init flag
  discrete_state->get_mutable_vector(is_init_idx_).SetAtIndex(0, 1);

  return EventStatus::Succeeded();
}

void DynamicResidualSystem::CopyOutput(const Context<double>& context,
                BasicVector<double>* residual) const {
  residual->SetFromVector(
      context.get_discrete_state().get_vector(residual_idx_).get_value());
}

}  // namespace systems
}  // namespace dairlib
