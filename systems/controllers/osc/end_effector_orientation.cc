#include "end_effector_orientation.h"
#include <iostream>
#include "dairlib/lcmt_radio_out.hpp"

using drake::systems::BasicVector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::systems::Context;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorOrientationTrajectoryGenerator::EndEffectorOrientationTrajectoryGenerator() {
  auto pp = drake::trajectories::PiecewiseQuaternionSlerp<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ = this->DeclareAbstractInputPort("lcmt_radio_out",
      drake::Value<dairlib::lcmt_radio_out>{}).get_index();

  // Set 1, 0, 0, 0 as default
  Eigen::Quaterniond identity_quat(1.0, 0.0, 0.0, 0.0);    
  std::vector<double> time_breaks = {0.0, 1.0};
  std::vector<Eigen::Quaterniond> quat_samples = {identity_quat, identity_quat};
  PiecewiseQuaternionSlerp<double> empty_slerp_traj(time_breaks, quat_samples);
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort("end_effector_orientation", traj_inst,
                                  &EndEffectorOrientationTrajectoryGenerator::CalcTraj)
      .get_index();
}

void EndEffectorOrientationTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& radio_out = this->EvalInputValue<dairlib::lcmt_radio_out>(
    context, radio_port_);

  auto* casted_traj =
    dynamic_cast<PiecewiseQuaternionSlerp<double>*>(traj);
  DRAKE_DEMAND(casted_traj != nullptr);


  //if (radio_out->channel[14] and track_orientation_) {  TODO: Figure out why teleop tracks end effector orientation
  if (track_orientation_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context, trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    const auto* traj =
      dynamic_cast<const PiecewiseQuaternionSlerp<double>*>(&trajectory_input);
    *casted_traj = *traj; 

    // std::cout << "orientation time range: ["
    //         << trajectory_input.start_time() << ", "
    //         << trajectory_input.end_time() << "]\n";

    
  } else {
    std::cout << "0 orientation traj" << std::endl;

    PiecewiseQuaternionSlerp<double> result;
    Eigen::VectorXd neutral_quaternion = VectorXd::Zero(4);
    neutral_quaternion(0) = 1;
    result = drake::trajectories::PiecewiseQuaternionSlerp<double>(
        {0, 1.0},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
    *casted_traj = result;
  }
}

}  // namespace dairlib
