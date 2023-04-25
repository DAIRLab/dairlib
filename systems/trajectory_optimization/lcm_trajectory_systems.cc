#include "lcm_trajectory_systems.h"

#include <iostream>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/geometry/rgba.h"

namespace dairlib {
namespace systems {

using drake::geometry::Rgba;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
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
  lcm_traj_ =
      LcmTrajectory(dairlib::FindResourceOrThrow(default_trajectory_path_));
}

void LcmTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcm_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    lcm_traj_ = LcmTrajectory(lcm_traj->saved_traj);
  }
  const auto trajectory_block = lcm_traj_.GetTrajectory(trajectory_name_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (trajectory_block.datapoints.rows() == 3) {
    *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
  } else {
    *casted_traj = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
        trajectory_block.datapoints.bottomRows(3));
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
  lcm_traj_ =
      LcmTrajectory(dairlib::FindResourceOrThrow(default_trajectory_path_));
  lcm_traj_.GetTrajectory(trajectory_name_);
}

void LcmOrientationTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcm_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    lcm_traj_ = LcmTrajectory(lcm_traj->saved_traj);
    try {
      lcm_traj_.GetTrajectory(trajectory_name_);
    } catch (std::exception& e) {
      std::cerr << "Make sure the planner is sending orientation" << std::endl;
      throw std::out_of_range("");
    }
  }
  const auto trajectory_block = lcm_traj_.GetTrajectory(trajectory_name_);
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
  for (int i = 0; i < trajectory_block.datapoints.cols(); ++i) {
    quaternion_datapoints.push_back(
        drake::math::RollPitchYaw<double>(trajectory_block.datapoints.col(i))
            .ToQuaternion());
  }
  *casted_traj = PiecewiseQuaternionSlerp(
      CopyVectorXdToStdVector(trajectory_block.time_vector),
      quaternion_datapoints);
}

LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name, const std::string& default_trajectory_path)
    : meshcat_(meshcat),
      trajectory_name_(std::move(trajectory_name)),
      default_trajectory_path_(default_trajectory_path) {
  this->set_name(trajectory_name);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  lcm_traj_ =
      LcmTrajectory(dairlib::FindResourceOrThrow(default_trajectory_path_));
  DeclarePerStepDiscreteUpdateEvent(&LcmTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcm_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    lcm_traj_ = LcmTrajectory(lcm_traj->saved_traj);
  }
  const auto trajectory_block = lcm_traj_.GetTrajectory(trajectory_name_);
  MatrixXd line_points = MatrixXd::Zero(3, N_);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, trajectory_block.time_vector[0],
                          trajectory_block.time_vector.tail(1)[0]);
  if (trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
      line_points(2, i) += 0.7645;
    }
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
        trajectory_block.datapoints.bottomRows(3));
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
      line_points(2, i) += 0.7645;
    }
  }

  DRAKE_DEMAND(line_points.rows() == 3);
  meshcat_->SetLine("/trajectories/" + trajectory_name_, line_points, 100,
                    rgba_);

  if (lcm_traj_.HasTrajectory("end_effector_orientation_target")) {
    const auto orientation_block =
        lcm_traj_.GetTrajectory("end_effector_orientation_target");
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        orientation_block.time_vector, orientation_block.datapoints.topRows(3),
        orientation_block.datapoints.bottomRows(3));
    for (int i = 0; i < line_points.cols(); ++i) {
      auto pose = drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(trajectory.value(breaks(i))),
          line_points.col(i));
      auto box = drake::geometry::Box(0.1, 0.1, 0.01);
      auto rgba_transparent = rgba_;
      rgba_transparent.set(rgba_.r(),
                           rgba_.g(),
                           rgba_.b(),
                           (line_points.cols() - double(i)) / line_points.cols()
                               * rgba_.a());
      meshcat_->SetObject("/trajectories/end_effector_pose/" + std::to_string(i), box, rgba_transparent);
      meshcat_->SetTransform("/trajectories/end_effector_pose/" + std::to_string(i), pose);
    }
  }
//  if (lcm_traj_.HasTrajectory("object_orientation_target")) {
//    const auto orientation_block =
//        lcm_traj_.GetTrajectory("object_orientation_target");
//    for (int i = 0; i < orientation_block.datapoints.cols(); ++i) {
//      auto pose = drake::math::RigidTransform<double>(
//          Quaterniond(orientation_block.datapoints(0, i),
//                      orientation_block.datapoints(1, i),
//                      orientation_block.datapoints(2, i),
//                      orientation_block.datapoints(3, i)),
//          line_points.col(i));
//      auto box = drake::geometry::Box(0.1, 0.1, 0.01);
//      meshcat_->SetObject("/trajectories/object" + std::to_string(i), box, rgba_);
//      meshcat_->SetTransform("/trajectories/object" + std::to_string(i), pose);
//    }
//  }

  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
