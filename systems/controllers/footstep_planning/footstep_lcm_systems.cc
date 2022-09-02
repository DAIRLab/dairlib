#include "footstep_lcm_systems.h"

namespace dairlib::systems::controllers{

using Eigen::Vector3d;

FootstepSender::FootstepSender() {
  this->DeclareVectorInputPort("footstep", 3);
  this->DeclareAbstractOutputPort(
      "footstep_lcm_msg", dairlib::lcmt_footstep_target(),
      &FootstepSender::CopyMessage);
}

void FootstepSender::CopyMessage(const drake::systems::Context<double> &context,
                                 dairlib::lcmt_footstep_target *msg) const {
  Vector3d target = this->EvalVectorInput(context, 0)->get_value();
  for (int i = 0; i < 3; i++) {
    msg->target[i] = target(i);
  }
}

FootstepReceiver::FootstepReceiver() {
  this->DeclareAbstractInputPort("footstep_lcm_msg",
                                 drake::Value<dairlib::lcmt_footstep_target>());
  this->DeclareVectorOutputPort("footstep", 3, &FootstepReceiver::CopyMessage);
}

void FootstepReceiver::CopyMessage(const drake::systems::Context<double> &context,
                                   drake::systems::BasicVector<double> *y) const {
  const auto msg = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(msg != nullptr);
  Vector3d target = Vector3d::Map(
      msg->get_value<dairlib::lcmt_footstep_target>().target);
  y->set_value(target);
}

}