#include "examples/magna/systems/utils/simulation_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_schunk_wsg_command.hpp"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace simulation_utils {

using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FrankaHandCommandToTrajectory::FrankaHandCommandToTrajectory() {
  input_franka_hand_command_port_ =
      this->DeclareAbstractInputPort(
              "franka_hand_command",
              drake::Value<drake::lcmt_schunk_wsg_command>{})
          .get_index();
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& default_instantiation = pp;
  hand_position_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "hand_position_trajectory", default_instantiation,
              &FrankaHandCommandToTrajectory::CalcHandPositionTrajectory)
          .get_index();
}

void FrankaHandCommandToTrajectory::CalcHandPositionTrajectory(
    const Context<double>& context, Trajectory<double>* output) const {
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output);
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& franka_hand_command =
      input->get_value<drake::lcmt_schunk_wsg_command>();
  MatrixXd finger_offsets(2, 1);
  // Convert from mm to m and from gripper width to finger offsets
  finger_offsets << -franka_hand_command.target_position_mm / 2000.0,
      franka_hand_command.target_position_mm / 2000.0;
  *casted_traj = PiecewisePolynomial<double>(finger_offsets);
}
}  // namespace simulation_utils
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
