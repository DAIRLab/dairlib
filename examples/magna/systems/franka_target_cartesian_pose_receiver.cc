#include "franka_target_cartesian_pose_receiver.h"

#include "dairlib/lcmt_franka_cartesian_pose.hpp"

using drake::systems::Context;
using Eigen::VectorXd;

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

FrankaTargetCartesianPoseReceiver::FrankaTargetCartesianPoseReceiver(
    double expire_time)
    : expire_time_(expire_time) {
  this->DeclareAbstractInputPort(
      "lcmt_franka_cartesian_pose",
      drake::Value<dairlib::lcmt_franka_cartesian_pose>{});
  this->DeclareVectorOutputPort(
      "target_cartesian_pose", BasicVector<double>(6),
      &FrankaTargetCartesianPoseReceiver::CopyInputOut, {all_sources_ticket()});
}

void FrankaTargetCartesianPoseReceiver::CopyInputOut(
    const Context<double>& context, BasicVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg =
      input->get_value<dairlib::lcmt_franka_cartesian_pose>();

  if (input_msg.utime != 0)
    DRAKE_THROW_UNLESS(context.get_time() - input_msg.utime * 1e-6 <
                       expire_time_);

  VectorXd input_vector = VectorXd::Zero(6);

  for (int i = 0; i < 6; i++) {
    input_vector(i) = input_msg.target_cartesian_pose[i];
  }
  output->SetFromVector(input_vector);
}
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
