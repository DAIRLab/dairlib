#include "plate_balancing_target.h"
#include <iostream>
#include "dairlib/lcmt_radio_out.hpp"

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using dairlib::systems::StateVector;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

PlateBalancingTargetGenerator::PlateBalancingTargetGenerator(const MultibodyPlant<double>& object_plant) {
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
}

void PlateBalancingTargetGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& neutral_pose, double x_scale, double y_scale,
    double z_scale) {
  neutral_pose_ = neutral_pose;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void PlateBalancingTargetGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  VectorXd y0 = neutral_pose_;
  // Update target if remote trigger is active
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
  const StateVector<double>* tray_state =
      (StateVector<double>*)this->EvalVectorInput(context, tray_state_port_);
  VectorXd target_tray_state = VectorXd::Zero(7);
  VectorXd tray_position = neutral_pose_;
  tray_position[2] += 0.015; // thickness of end effector and tray
  tray_position(0) += radio_out->channel[0] * x_scale_;
  tray_position(1) += radio_out->channel[1] * y_scale_;
  tray_position(2) += radio_out->channel[2] * z_scale_;
  if ((tray_state->GetPositions().tail(3) - tray_position).norm() < 0.1){
    tray_position[2] += 0.1; // raise the tray once it is close
  }
  target_tray_state << 1, 0, 0, 0, tray_position;
  target->SetFromVector(target_tray_state);
}

}  // namespace systems
}  // namespace dairlib