#include "plate_balancing_target.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"

using dairlib::systems::StateVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

PlateBalancingTargetGenerator::PlateBalancingTargetGenerator(
    const MultibodyPlant<double>& object_plant, double target_threshold)
    : target_threshold_(target_threshold) {
  // Input/Output Setup
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  tray_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(object_plant.num_positions(),
                                              object_plant.num_velocities()))
          .get_index();
  end_effector_target_port_ =
      this->DeclareVectorOutputPort(
              "end_effector_target", BasicVector<double>(3),
              &PlateBalancingTargetGenerator::CalcEndEffectorTarget)
          .get_index();
  tray_target_port_ = this->DeclareVectorOutputPort(
                              "tray_target", BasicVector<double>(7),
                              &PlateBalancingTargetGenerator::CalcTrayTarget)
                          .get_index();
  reached_first_target_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  DeclareForcedDiscreteUpdateEvent(
      &PlateBalancingTargetGenerator::DiscreteVariableUpdate);
}

EventStatus PlateBalancingTargetGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const StateVector<double>* tray_state =
      (StateVector<double>*)this->EvalVectorInput(context, tray_state_port_);
  if (context.get_discrete_state(reached_first_target_idx_)[0] == 0 &&
      (tray_state->GetPositions().tail(3) - first_target_).norm() <
          target_threshold_) {
    discrete_state->get_mutable_value(reached_first_target_idx_)[0] = 1;
  }
  if ((tray_state->GetPositions().tail(3) - second_target_).norm() < 0.02) {
    discrete_state->get_mutable_value(reached_first_target_idx_)[0] = 2;
  }
  return EventStatus::Succeeded();
}

void PlateBalancingTargetGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& initial_pose, const Eigen::Vector3d& first_target,
    const Eigen::Vector3d& second_target, double x_scale, double y_scale,
    double z_scale) {
  initial_pose_ = initial_pose;
  first_target_ = first_target;
  second_target_ = second_target;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void PlateBalancingTargetGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);

  VectorXd y0 = first_target_;
  // Update target if remote trigger is active
  if (context.get_discrete_state(reached_first_target_idx_)[0] == 1) {
    y0 = second_target_;  // raise the tray once it is close
    y0[2] -= 0.015;
  } else if (context.get_discrete_state(reached_first_target_idx_)[0] == 2) {
    y0 = initial_pose_;  // raise the tray once it is close
    y0[2] -= 0.015;
    y0[0] -= 0.1;
  }
  if (radio_out->channel[13] > 0) {
    y0(0) += radio_out->channel[0] * x_scale_;
    y0(1) += radio_out->channel[1] * y_scale_;
    y0(2) += radio_out->channel[2] * z_scale_;
  }
  target->SetFromVector(y0);
}

void PlateBalancingTargetGenerator::CalcTrayTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  VectorXd target_tray_state = VectorXd::Zero(7);
  VectorXd tray_position = first_target_;
  tray_position[2] += 0.015;  // thickness of end effector and tray

  if (context.get_discrete_state(reached_first_target_idx_)[0] == 1) {
    tray_position = second_target_;  // raise the tray once it is close
  } else if (context.get_discrete_state(reached_first_target_idx_)[0] == 2) {
    tray_position = initial_pose_;  // raise the tray once it is close
  }
  tray_position(0) += radio_out->channel[0] * x_scale_;
  tray_position(1) += radio_out->channel[1] * y_scale_;
  tray_position(2) += radio_out->channel[2] * z_scale_;
  target_tray_state << 1, 0, 0, 0, tray_position;
  target->SetFromVector(target_tray_state);
}

}  // namespace systems
}  // namespace dairlib