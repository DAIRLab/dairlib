#include "end_effector_orientation.h"

#include "dairlib/lcmt_radio_out.hpp"

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
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort("end_effector_orientation", traj_inst,
                                  &EndEffectorOrientationGenerator::CalcTraj)
      .get_index();
}

PiecewiseQuaternionSlerp<double> EndEffectorOrientationGenerator::GeneratePose(
    const drake::systems::Context<double>& context) const {
  Eigen::VectorXd neutral_quaternion = VectorXd::Zero(4);
  neutral_quaternion(0) = 1;
  return drake::trajectories::PiecewiseQuaternionSlerp<double>(
      {0, 1}, {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
}

void EndEffectorOrientationGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  //  // Read in finite state machine

  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (radio_out->channel[14] and track_orientation_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context, trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    *casted_traj = *(PiecewiseQuaternionSlerp<double>*)dynamic_cast<
        const PiecewiseQuaternionSlerp<double>*>(&trajectory_input);
  } else {
    *casted_traj = GeneratePose(context);
  }
}

}  // namespace dairlib
