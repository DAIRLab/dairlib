#include "end_effector_force.h"
#include <iostream>
#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::RigidBodyFrame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorForceTrajectoryGenerator::EndEffectorForceTrajectoryGenerator()
  : n_forces_(3) {
  EndEffectorForceTrajectoryGenerator(3);
}

EndEffectorForceTrajectoryGenerator::EndEffectorForceTrajectoryGenerator(int n_forces) 
  : n_forces_(n_forces) {
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  std::cout << "ee force initialized" << std::endl;

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ = this->DeclareAbstractInputPort("lcmt_radio_out",
      drake::Value<dairlib::lcmt_radio_out>{}).get_index();
  controller_switch_index_ = this->DeclareDiscreteState(VectorXd::Ones(1));
  DeclareForcedDiscreteUpdateEvent(
      &EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate);
  PiecewisePolynomial<double> empty_pp_traj(VectorXd::Zero(n_forces_));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort(
      "end_effector_force_trajectory", traj_inst,
      &EndEffectorForceTrajectoryGenerator::CalcTraj);
}

EventStatus EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& radio_out = this->EvalInputValue<dairlib::lcmt_radio_out>(
    context, radio_port_);
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  bool using_c3 = context.get_discrete_state(controller_switch_index_)[0];
  if (!using_c3 && radio_out->channel[14] == 0) {
    if (!trajectory_input.value(0).isZero() &&
        (context.get_time() - trajectory_input.start_time()) < 0.04) {
      discrete_state->get_mutable_value(controller_switch_index_)[0] = 1;
    }
  }
  return EventStatus::Succeeded();
}

void EndEffectorForceTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  //  // Read in finite state machine
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  const auto& radio_out = this->EvalInputValue<dairlib::lcmt_radio_out>(
    context, radio_port_);

  // std::cout << "force time range: ["
  //         << trajectory_input.start_time() << ", "
  //         << trajectory_input.end_time() << "]\n";

  auto* casted_traj =
    dynamic_cast<PiecewisePolynomial<double>*>(traj);
  DRAKE_DEMAND(casted_traj != nullptr);

  if (radio_out->channel[11] || radio_out->channel[14] ||
      trajectory_input.value(0).isZero()) {
    auto zero_traj =
        PiecewisePolynomial<double>::FirstOrderHold({0.0, 1.0}, 
          {Eigen::VectorXd::Zero(n_forces_), Eigen::VectorXd::Zero(n_forces_)});
    *casted_traj = zero_traj; 
  } else {
    if (context.get_discrete_state(controller_switch_index_)[0]) {
      const auto* traj =
        dynamic_cast<const PiecewisePolynomial<double>*>(&trajectory_input);
      *casted_traj = *traj; 
    }
  }
  // std::cout << "force time range casted: ["
  //     << casted_traj->start_time() << ", "
  //     << casted_traj->end_time() << "]\n";
}

}  // namespace dairlib
