#include "end_effector_force_trajectory.h"

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorForceTrajectoryGenerator::EndEffectorForceTrajectoryGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context)
    : plant_(plant), context_(context), world_(plant.world_frame()) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x", OutputVector<double>(plant_.num_positions(),
                                                  plant_.num_velocities(),
                                                  plant_.num_actuators()))
                    .get_index();
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  controller_switch_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  DeclareForcedDiscreteUpdateEvent(
      &EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate);
  PiecewisePolynomial<double> empty_pp_traj(Vector3d::Zero());
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort(
      "end_effector_force_trajectory", traj_inst,
      &EndEffectorForceTrajectoryGenerator::CalcTraj);
}


EventStatus EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {

  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  bool using_c3 = context.get_discrete_state(controller_switch_index_)[0];
  if (!using_c3 && radio_out->channel[14] == 0) {
    if (!trajectory_input.value(0).isZero() && (context.get_time() - trajectory_input.start_time()) < 0.04) {
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
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (radio_out->channel[11] || radio_out->channel[14] || trajectory_input.value(0).isZero()) {
    *casted_traj = drake::trajectories::PiecewisePolynomial<double>(Vector3d::Zero());
  } else {
    if (context.get_discrete_state(controller_switch_index_)[0]){
      *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
          const PiecewisePolynomial<double>*>(&trajectory_input);
    }
  }
}

}  // namespace dairlib
