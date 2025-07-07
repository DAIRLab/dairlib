#include "lcm_trajectory_systems.h"

#include <iostream>

#include "common/eigen_utils.h"
#include "common/find_resource.h"

#include "drake/common/schema/rotation.h"
#include "drake/geometry/rgba.h"

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

namespace dairlib {
namespace systems {

// Receiver system for C3 LCM trajectory messages.
// Handles both polynomial and quaternion (orientation) trajectories.
LcmC3TrajectoryReceiver::LcmC3TrajectoryReceiver(std::string trajectory_name,
                                                 bool is_quaternion,
                                                 bool has_velocity)
    : trajectory_name_(std::move(trajectory_name)),
      is_quaternion_(is_quaternion),
      has_velocity_(has_velocity) {
  this->set_name(trajectory_name_);
  // Declare input port for C3 trajectory LCM messages.
  lcm_trajectory_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_trajectory",
                                     drake::Value<c3::lcmt_c3_trajectory>{})
          .get_index();

  if (is_quaternion_) {
    // Use PiecewiseQuaternionSlerp for orientation trajectories.
    trajectory_output_port_ =
        this->DeclareAbstractOutputPort(
                trajectory_name_, PiecewiseQuaternionSlerp<double>(),
                &LcmC3TrajectoryReceiver::OutputQuaternionTrajectory)
            .get_index();
  } else {
    // Use PiecewisePolynomial for standard trajectories.
    trajectory_output_port_ =
        this->DeclareAbstractOutputPort(
                trajectory_name_, PiecewisePolynomial<double>(),
                &LcmC3TrajectoryReceiver::OutputTrajectory)
            .get_index();
  }
}

// Outputs a PiecewisePolynomial trajectory reconstructed from the LCM message.
void LcmC3TrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    PiecewisePolynomial<double>* traj) const {
  const auto& lcmt_c3_traj = this->EvalInputValue<c3::lcmt_c3_trajectory>(
      context, lcm_trajectory_input_port_);
  // If message is invalid or missing, output a zero trajectory.
  if (lcmt_c3_traj->utime <= 1e-3) {
    return;
  }

  // Parse and reconstruct the trajectory from the message.
  auto lcm_traj = LcmTrajectory(*lcmt_c3_traj);
  const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);
  if (!has_velocity_) {
    *traj = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
  } else {
    int n_r = trajectory_block.datapoints.rows();
    *traj = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block.time_vector,
        trajectory_block.datapoints.topRows(n_r / 2),
        trajectory_block.datapoints.bottomRows(n_r / 2));
  }
}

// Outputs a PiecewiseQuaternionSlerp trajectory reconstructed from the LCM
// message.
void LcmC3TrajectoryReceiver::OutputQuaternionTrajectory(
    const drake::systems::Context<double>& context,
    PiecewiseQuaternionSlerp<double>* traj) const {
  const auto& lcmt_c3_traj = this->EvalInputValue<c3::lcmt_c3_trajectory>(
      context, lcm_trajectory_input_port_);

  if (lcmt_c3_traj->utime <= 1e-3) {
    return;
  }
  auto lcm_traj = LcmTrajectory(*lcmt_c3_traj);
  lcm_traj.GetTrajectory(trajectory_name_);

  const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);
  // Convert each column (RPY) to a quaternion.
  std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
  for (int i = 0; i < trajectory_block.datapoints.cols(); ++i) {
    if (trajectory_block.datapoints.rows() == 3 ||
        (trajectory_block.datapoints.rows() == 6 && has_velocity_)) {
      quaternion_datapoints.push_back(
          drake::math::RollPitchYaw<double>(
              trajectory_block.datapoints.topRows(3).col(i))
              .ToQuaternion());
    } else if (trajectory_block.datapoints.rows() == 4) {
      Eigen::Vector4d quat = trajectory_block.datapoints.topRows(4).col(i);
      quaternion_datapoints.push_back(
          !quat.isZero() ? Eigen::Quaternion<double>(quat)
                         : Eigen::Quaternion<double>(1, 0, 0, 0));
    } else {
      throw std::runtime_error(
          "Invalid trajectory data points size: " +
          std::to_string(trajectory_block.datapoints.rows()));
    }
  }
  *traj = PiecewiseQuaternionSlerp(
      CopyVectorXdToStdVector(trajectory_block.time_vector),
      quaternion_datapoints);
}

}  // namespace systems
}  // namespace dairlib
