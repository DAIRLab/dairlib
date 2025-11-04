#include "systems/franka_hand_target_position_receiver.h"

#include "dairlib/lcmt_franka_hand_target_position.hpp"

using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {
FrankaHandTargetPositionReceiver::FrankaHandTargetPositionReceiver() {
  this->DeclareAbstractInputPort(
      "lcmt_franka_hand_target_position",
      drake::Value<dairlib::lcmt_franka_hand_target_position>{});
  this->DeclareAbstractOutputPort(
      "hand_target_position",
      []() {
        return drake::AbstractValue::Make<Trajectory<double>>(
            PiecewisePolynomial<double>(VectorXd::Zero(1)));
      },
      [this](const Context<double>& context, drake::AbstractValue* traj) {
        this->CalcTraj(context, traj);
      });
}

void FrankaHandTargetPositionReceiver::CalcTraj(
    const Context<double>& context, drake::AbstractValue* traj) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg =
      input->get_value<dairlib::lcmt_franka_hand_target_position>();
  auto output_value = &traj->get_mutable_value<Trajectory<double>>();
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output_value);
  *casted_traj = PiecewisePolynomial<double>(input_msg.hand_target_position[0] *
                                             Eigen::VectorXd::Ones(1));
}
}  // namespace systems
}  // namespace dairlib
