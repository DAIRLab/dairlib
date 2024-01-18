#include "lcm_trajectory_systems.h"

#include <iostream>


#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/common/schema/rotation.h"
#include "drake/geometry/rgba.h"

namespace dairlib {
namespace systems {

using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

LcmTrajectoryReceiver::LcmTrajectoryReceiver(std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(trajectory_name_, traj_inst,
                                      &LcmTrajectoryReceiver::OutputTrajectory)
          .get_index();
}

void LcmTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcmt_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
    const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);

    if (trajectory_block.datapoints.rows() == 3) {
      *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
          trajectory_block.time_vector, trajectory_block.datapoints);
      //      *casted_traj = PiecewisePolynomial<double>::ZeroOrderHold(
      //          trajectory_block.time_vector, trajectory_block.datapoints);
    } else {
      *casted_traj = PiecewisePolynomial<double>::CubicHermite(
          trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
          trajectory_block.datapoints.bottomRows(3));
    }
  } else {
    *casted_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}

LcmOrientationTrajectoryReceiver::LcmOrientationTrajectoryReceiver(
    std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(
              trajectory_name_, traj_inst,
              &LcmOrientationTrajectoryReceiver::OutputTrajectory)
          .get_index();
}

void LcmOrientationTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcmt_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
    try {
      lcm_traj.GetTrajectory(trajectory_name_);
    } catch (std::exception& e) {
      std::cerr << "Make sure the planner is sending orientation" << std::endl;
      throw std::out_of_range("");
    }
    const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);

    std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
    for (int i = 0; i < trajectory_block.datapoints.cols(); ++i) {
      quaternion_datapoints.push_back(
          drake::math::RollPitchYaw<double>(trajectory_block.datapoints.col(i))
              .ToQuaternion());
    }
    *casted_traj = PiecewiseQuaternionSlerp(
        CopyVectorXdToStdVector(trajectory_block.time_vector),
        quaternion_datapoints);
  } else {
    *casted_traj = drake::trajectories::PiecewiseQuaternionSlerp<double>(
        {0, 1},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
  }
}

}  // namespace systems
}  // namespace dairlib
