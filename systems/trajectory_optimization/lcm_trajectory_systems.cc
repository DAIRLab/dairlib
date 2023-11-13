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

LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name(trajectory_name_);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  //  lcm_traj_ =
  //      LcmTrajectory(dairlib::FindResourceOrThrow(default_trajectory_path_));
  DeclarePerStepDiscreteUpdateEvent(&LcmTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);
  MatrixXd line_points = MatrixXd::Zero(3, N_);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, trajectory_block.time_vector[0],
                          trajectory_block.time_vector.tail(1)[0]);
  if (trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
        trajectory_block.datapoints.bottomRows(3));
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  }

  DRAKE_DEMAND(line_points.rows() == 3);
  meshcat_->SetLine("/trajectories/" + trajectory_name_, line_points, 100,
                    rgba_);
  return drake::systems::EventStatus::Succeeded();
}

LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file,
    const std::string& translation_trajectory_name,
    const std::string& orientation_trajectory_name, int num_poses)
    : meshcat_(meshcat),
      translation_trajectory_name_(translation_trajectory_name),
      orientation_trajectory_name_(orientation_trajectory_name),
      N_(num_poses) {
  this->set_name("/poses/" + model_file);

  multipose_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, N_, 1.0 * VectorXd::LinSpaced(N_, 0, 0.4), "", meshcat);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmPoseDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmPoseDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  MatrixXd object_poses = MatrixXd::Zero(7, N_);

  const auto& lcm_translation_traj =
      lcm_traj.GetTrajectory(translation_trajectory_name_);
  auto translation_trajectory = PiecewisePolynomial<double>::CubicHermite(
      lcm_translation_traj.time_vector,
      lcm_translation_traj.datapoints.topRows(3),
      lcm_translation_traj.datapoints.bottomRows(3));
  auto orientation_trajectory = PiecewiseQuaternionSlerp<double>(
      {0, 1}, {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});

  if (lcm_traj.HasTrajectory(orientation_trajectory_name_)) {
    const auto& lcm_orientation_traj =
        lcm_traj.GetTrajectory(orientation_trajectory_name_);
    std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
    for (int i = 0; i < lcm_orientation_traj.datapoints.cols(); ++i) {
      VectorXd orientation_sample = lcm_orientation_traj.datapoints.col(i);
      if (orientation_sample.isZero()) {
        quaternion_datapoints.push_back(Quaterniond(1, 0, 0, 0));
      } else {
        quaternion_datapoints.push_back(
            Quaterniond(orientation_sample[0], orientation_sample[1],
                        orientation_sample[2], orientation_sample[3]));
      }
    }
    orientation_trajectory = PiecewiseQuaternionSlerp(
        CopyVectorXdToStdVector(lcm_orientation_traj.time_vector),
        quaternion_datapoints);
  }

  // ASSUMING orientation and translation trajectories have the same breaks
  VectorXd translation_breaks =
      VectorXd::LinSpaced(N_, lcm_translation_traj.time_vector[0],
                          lcm_translation_traj.time_vector.tail(1)[0]);
  for (int i = 0; i < object_poses.cols(); ++i) {
    object_poses.col(i) << orientation_trajectory.value(translation_breaks(i)),
        translation_trajectory.value(translation_breaks(i));
  }

  multipose_visualizer_->DrawPoses(object_poses);

  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
