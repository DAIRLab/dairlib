/**This system is used by the osc diagram to read an end effector trajectory from an lcm message and pass it onto the OSC class.
* This system is also present under examples/franka/systems but has been copied here to be used by the instance of the osc
* in the sampling_c3 controller examples. */
#include "end_effector_trajectory.h"
#include <iostream>
#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
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
    const MultibodyPlant<double>& plant, Context<double>* context,
    const Eigen::VectorXd& neutral_pose, bool teleop_neutral_pose,
    const std::string& end_effector_name)
    : plant_(plant), context_(context), end_effector_name_(end_effector_name) {
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
  radio_port_ = this->DeclareAbstractInputPort("lcmt_radio_out",
      drake::Value<dairlib::lcmt_radio_out>{}).get_index();

  PiecewisePolynomial<double> empty_pp_traj(neutral_pose);
  Trajectory<double>& traj_inst = empty_pp_traj;
  if (teleop_neutral_pose) {
    this->DeclareAbstractOutputPort(
      "end_effector_trajectory", traj_inst,
      &EndEffectorTrajectoryGenerator::CalcPoseShiftingTraj);
  }
  else {
    this->DeclareAbstractOutputPort(
      "end_effector_trajectory", traj_inst,
      &EndEffectorTrajectoryGenerator::CalcNeutralPoseBasedTraj);
  }
}

void EndEffectorTrajectoryGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& neutral_pose, double x_scale, double y_scale,
    double z_scale) {
  neutral_pose_ = neutral_pose;
  shifting_pose_ = neutral_pose;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void EndEffectorTrajectoryGenerator::CalcNeutralPoseBasedTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  const auto& radio_out = this->EvalInputValue<dairlib::lcmt_radio_out>(
    context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (radio_out->channel[14]) {
    PiecewisePolynomial<double> result;

    // Compute the target position based on an offset from neutral pose.
    VectorXd y_0 = neutral_pose_;
    y_0(0) += radio_out->channel[0] * x_scale_;
    y_0(1) += radio_out->channel[1] * y_scale_;
    y_0(2) += radio_out->channel[2] * z_scale_;

    result = drake::trajectories::PiecewisePolynomial<double>(y_0);
    *casted_traj = result;
  } else {
    if (trajectory_input.value(0).isZero()) {
    } else {
      *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
          const PiecewisePolynomial<double>*>(&trajectory_input);
    }
  }
}

void EndEffectorTrajectoryGenerator::CalcPoseShiftingTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  const auto& radio_out = this->EvalInputValue<dairlib::lcmt_radio_out>(
    context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (radio_out->channel[14]) {
    if (!was_in_teleop_mode_) {
      // Update the shifting position to the current position -- this means
      // every time teleop mode is newly entered, the robot will stay where it
      // is instead of snapping to the neutral position elsewhere.
      const OutputVector<double>* franka_state =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
      VectorXd q_franka = franka_state->GetPositions();
      multibody::SetPositionsIfNew<double>(plant_, q_franka,
                                           context_);
      auto end_effector_pose = plant_.EvalBodyPoseInWorld(
        *context_, plant_.GetBodyByName(end_effector_name_));
      shifting_pose_ = end_effector_pose.translation();
    }
    was_in_teleop_mode_ = true;

    PiecewisePolynomial<double> result;

    // Compute the target position by stepping the held pose.
    if (std::abs(radio_out->channel[0]) > 0.01) {
      shifting_pose_(0) += radio_out->channel[0] * x_scale_;
    }
    if (std::abs(radio_out->channel[1]) > 0.01) {
      shifting_pose_(1) += radio_out->channel[1] * y_scale_;
    }
    if (std::abs(radio_out->channel[2]) > 0.01) {
      shifting_pose_(2) += radio_out->channel[2] * z_scale_;
    }
    VectorXd y_0 = shifting_pose_;

    result = drake::trajectories::PiecewisePolynomial<double>(y_0);
    *casted_traj = result;
  }
  else {
    was_in_teleop_mode_ = false;
    if (trajectory_input.value(0).isZero()) {
    } else {
      *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
          const PiecewisePolynomial<double>*>(&trajectory_input);
    }
  }
}

}  // namespace dairlib
