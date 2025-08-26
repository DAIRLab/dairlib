#include "end_effector_orientation.h"

#include "dairlib/lcmt_radio_out.hpp"

using drake::systems::BasicVector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::systems::Context;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

// Implements a trajectory generator for end effector orientation.
// Selects between a neutral orientation and a provided trajectory
// based on radio input and tracking flag.
EndEffectorOrientationTrajectoryGenerator::
    EndEffectorOrientationTrajectoryGenerator() {
  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory", drake::Value<PiecewiseQuaternionSlerp<double>>())
          .get_index();
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort(
          "end_effector_orientation", traj_inst,
          &EndEffectorOrientationTrajectoryGenerator::CalcTraj)
      .get_index();
}

// Calculates the output orientation trajectory based on radio input.
// If tracking is enabled and the radio flag is set, outputs the input
// trajectory. Otherwise, outputs a neutral orientation.
void EndEffectorOrientationTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (radio_out->value()[14] and track_orientation_) {
    auto trajectory_input =
        this->EvalInputValue<PiecewiseQuaternionSlerp<double>>(
            context, trajectory_port_);
    *casted_traj = *trajectory_input;
  } else {
    PiecewiseQuaternionSlerp<double> result;
    result = drake::trajectories::PiecewiseQuaternionSlerp<double>(
        {0, 1},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
    *casted_traj = result;
  }
}

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
