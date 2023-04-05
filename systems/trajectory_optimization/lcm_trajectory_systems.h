#pragma once

#include <string>
#include <vector>

#include <drake/geometry/meshcat.h>

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and outputs it as a drake PiecewisePolynomial.
class LcmTrajectoryReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmTrajectoryReceiver(std::string trajectory_name);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_trajectory() const {
    return this->get_output_port(trajectory_output_port_);
  }

 private:
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;
  drake::systems::InputPortIndex trajectory_input_port_;
  drake::systems::OutputPortIndex trajectory_output_port_;
  const std::string trajectory_name_;

  mutable LcmTrajectory lcm_traj_;
  std::string nominal_stand_path_ =
      "examples/franka/saved_trajectories/default_end_effector_pose";
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class LcmTrajectoryDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmTrajectoryDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                               std::string trajectory_name);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

 private:
  void OutputTrajectory(const drake::systems::Context<double>& context,
                        drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::EventStatus DrawTrajectory(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string trajectory_name_;
  mutable LcmTrajectory lcm_traj_;
  std::string nominal_stand_path_ =
      "examples/franka/saved_trajectories/default_end_effector_pose";
};

}  // namespace systems
}  // namespace dairlib
