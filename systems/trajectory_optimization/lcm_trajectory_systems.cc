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
    *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
    if (trajectory_block.datapoints.rows() == 3) {
      *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
          trajectory_block.time_vector, trajectory_block.datapoints);
    } else {
      *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
          trajectory_block.time_vector,
          trajectory_block.datapoints.topRows(
              trajectory_block.datapoints.rows() / 2));
      //      *casted_traj = PiecewisePolynomial<double>::CubicHermite(
      //          trajectory_block.time_vector,
      //          trajectory_block.datapoints.topRows(3),
      //          trajectory_block.datapoints.bottomRows(3));
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
