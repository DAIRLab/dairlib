#include "lcm_visualization_systems.h"

#include <dairlib/lcmt_timestamped_saved_traj.hpp>

#include "common/eigen_utils.h"

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

LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name("LcmTrajectoryDrawer: " + trajectory_name_);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

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

LcmConfigurationDrawer::LcmConfigurationDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file,
    const std::string& configuration_trajectory_name, int num_poses,
    bool add_transparency)
    : meshcat_(meshcat),
      configuration_trajectory_name_(configuration_trajectory_name),
      N_(num_poses) {
  this->set_name("LcmConfigurationDrawer");

  Eigen::VectorXd alpha_scale;
  if (add_transparency) {
    alpha_scale = 1.0 * VectorXd::LinSpaced(N_ - 1, 0.2, 0.5);
  } else {
    alpha_scale = 1.0 * VectorXd::Ones(N_ - 1);
  }
  alpha_scale.reverseInPlace();

  multipose_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, N_ - 1, alpha_scale, "", meshcat);
  trajectory_input_port_ = this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmConfigurationDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmConfigurationDrawer::DrawTrajectory(
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

  if (lcm_traj.HasTrajectory(configuration_trajectory_name_)) {
    const auto& lcm_q_traj =
        lcm_traj.GetTrajectory(configuration_trajectory_name_);
    auto pp_traj = PiecewisePolynomial<double>::FirstOrderHold(
        lcm_q_traj.time_vector, lcm_q_traj.datapoints);
    const VectorXd breaks = VectorXd::LinSpaced(
        N_, lcm_q_traj.time_vector[0], lcm_q_traj.time_vector.tail(1)[0]);
    MatrixXd poses = MatrixXd::Zero(lcm_q_traj.datapoints.rows(), N_);

    for (int i = 0; i < N_; ++i) {
      poses.col(i) = pp_traj.value(breaks(i));
    }
    multipose_visualizer_->DrawPoses(poses);
  }

  return drake::systems::EventStatus::Succeeded();
}




}  // namespace systems
}  // namespace dairlib