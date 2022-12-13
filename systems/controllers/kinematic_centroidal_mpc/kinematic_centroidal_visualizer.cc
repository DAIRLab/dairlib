#include "kinematic_centroidal_visualizer.h"

#include <iostream>
#include <utility>

#include <dairlib/lcmt_saved_traj.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>

#include "lcm/lcm_trajectory.h"
#include "multibody/multipose_visualizer.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::systems::BasicVector;
using dairlib::systems::OutputVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;

namespace dairlib {

KinematicCentroidalVisualizer::KinematicCentroidalVisualizer(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::unique_ptr<multibody::MultiposeVisualizer> visualizer)
    : plant_(plant), context_(context), visualizer_(std::move(visualizer)) {
  this->set_name("kinematic_centroidal_visualizer");

  // Input/Output Setup
  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "kcmpc_trajectory",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(
      &KinematicCentroidalVisualizer::DrawTrajectory);
}

EventStatus KinematicCentroidalVisualizer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto* timestamped_lcm_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_port_);
  MatrixXd poses =
      MatrixXd::Zero(visualizer_->num_positions(), visualizer_->num_poses());
  std::cout << "drawing poses: " << std::endl;
  LcmTrajectory trajectory = LcmTrajectory(timestamped_lcm_traj->saved_traj);
  auto state_traj = trajectory.GetTrajectory("state_traj");
  for (int i = 0; i < poses.cols(); ++i) {
    int col_index = (int)(state_traj.datapoints.cols() * i / poses.cols());
    poses.col(i) =
        state_traj.datapoints.col(col_index).head(visualizer_->num_positions());
  }

  visualizer_->DrawPoses(poses);
  return EventStatus::Succeeded();
}

}  // namespace dairlib