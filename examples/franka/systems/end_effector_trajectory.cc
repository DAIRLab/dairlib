#include "end_effector_trajectory.h"
#include <iostream>
#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorTrajectoryGenerator::EndEffectorTrajectoryGenerator(
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
      &EndEffectorTrajectoryGenerator::DiscreteVariableUpdate);
  PiecewisePolynomial<double> empty_pp_traj(neutral_pose_);
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("end_effector_trajectory", traj_inst,
                                  &EndEffectorTrajectoryGenerator::CalcTraj);
}


EventStatus EndEffectorTrajectoryGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {

  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  bool using_c3 = context.get_discrete_state(controller_switch_index_)[0];
  if (!using_c3 && radio_out->channel[14] == 0) {
    if (!trajectory_input.value(0).isZero() && (context.get_time() - trajectory_input.start_time()) < 0.05) {
      discrete_state->get_mutable_value(controller_switch_index_)[0] = 1;
    }
  }
  return EventStatus::Succeeded();
}


void EndEffectorTrajectoryGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& neutral_pose, double x_scale, double y_scale,
    double z_scale) {
  neutral_pose_ = neutral_pose;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

PiecewisePolynomial<double> EndEffectorTrajectoryGenerator::GeneratePose(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  VectorXd y0 = neutral_pose_;
  y0(0) += radio_out->channel[0] * x_scale_;
  y0(1) += radio_out->channel[1] * y_scale_;
  y0(2) += radio_out->channel[2] * z_scale_;
  return drake::trajectories::PiecewisePolynomial<double>(y0);
}

void EndEffectorTrajectoryGenerator::CalcTraj(
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

  if (radio_out->channel[14]) {
    *casted_traj = GeneratePose(context);
  } else {
    if (trajectory_input.value(0).isZero()) {
//      *casted_traj = GeneratePose(context);
    } else {
      if (context.get_discrete_state(controller_switch_index_)[0]){
        *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
            const PiecewisePolynomial<double>*>(&trajectory_input);
      }
    }
  }
}

}  // namespace dairlib
