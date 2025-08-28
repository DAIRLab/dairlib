#include "systems/franka_target_joint_position_receiver.h"

#include "dairlib/lcmt_franka_target_joint_position.hpp"

using drake::systems::Context;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {
FrankaTargetJointPositionReceiver::FrankaTargetJointPositionReceiver() {
  this->DeclareAbstractInputPort(
      "lcmt_franka_target_joint_position",
      drake::Value<dairlib::lcmt_franka_target_joint_position>{});
  this->DeclareVectorOutputPort(
      "target_joint_position", BasicVector<double>(7),
      &FrankaTargetJointPositionReceiver::CopyInputOut, {all_sources_ticket()});
}

void FrankaTargetJointPositionReceiver::CopyInputOut(
    const Context<double>& context, BasicVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg =
      input->get_value<dairlib::lcmt_franka_target_joint_position>();

  VectorXd input_vector = VectorXd::Zero(7);

  for (int i = 0; i < 7; i++) {
    input_vector(i) = input_msg.target_joint_position[i];
  }
  output->SetFromVector(input_vector);
}
}  // namespace systems
}  // namespace dairlib
