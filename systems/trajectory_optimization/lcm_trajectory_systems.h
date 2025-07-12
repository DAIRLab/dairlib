#pragma once

#include <string>
#include <vector>

#include <c3/lcmt_c3_trajectory.hpp>
#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multipose_visualizer.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
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
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and outputs it as a drake PiecewisePolynomial.
class LcmOrientationTrajectoryReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmOrientationTrajectoryReceiver(std::string trajectory_name);

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
};

/**
 * @class LcmC3TrajectoryReceiver
 * @brief Drake LeafSystem for receiving and outputting LCM-based trajectories.
 *
 * The LcmC3TrajectoryReceiver class subscribes to LCM messages containing
 * trajectory data and provides the received trajectory as an output port. It
 * supports both standard and quaternion-based trajectories, depending on the
 * configuration.
 *
 * @note This class is intended for use within the Drake systems framework and
 * is typically used in trajectory optimization and execution pipelines.
 *
 * @see drake::systems::LeafSystem
 */
class LcmC3TrajectoryReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmC3TrajectoryReceiver(std::string trajectory_name,
                                   bool is_quaternion = false,
                                   bool has_velocity = false);

  const drake::systems::InputPort<double>& get_input_port_lcm_trajectory()
      const {
    return this->get_input_port(lcm_trajectory_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_trajectory() const {
    return this->get_output_port(trajectory_output_port_);
  }

 private:
  void OutputTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::PiecewisePolynomial<double>* traj) const;

  void OutputQuaternionTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::PiecewiseQuaternionSlerp<double>* traj) const;

  drake::systems::InputPortIndex lcm_trajectory_input_port_;
  drake::systems::OutputPortIndex trajectory_output_port_;
  const std::string trajectory_name_;
  bool is_quaternion_;
  bool has_velocity_;
};

}  // namespace systems
}  // namespace dairlib
