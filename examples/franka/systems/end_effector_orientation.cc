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

EndEffectorOrientationGenerator::EndEffectorOrientationGenerator() {
  auto pp = drake::trajectories::PiecewiseQuaternionSlerp<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort("end_effector_orientation", traj_inst,
                                  &EndEffectorOrientationGenerator::CalcTraj)
      .get_index();
}

void EndEffectorOrientationGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (radio_out->value()[14] and track_orientation_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context, trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    *casted_traj = *(PiecewiseQuaternionSlerp<double>*)dynamic_cast<
        const PiecewiseQuaternionSlerp<double>*>(&trajectory_input);
  } else {
    PiecewiseQuaternionSlerp<double> result;
    Eigen::VectorXd neutral_quaternion = VectorXd::Zero(4);
    neutral_quaternion(0) = 1;
    result = drake::trajectories::PiecewiseQuaternionSlerp<double>(
        {0, 1},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
    *casted_traj = result;
  }
}

}  // namespace dairlib
