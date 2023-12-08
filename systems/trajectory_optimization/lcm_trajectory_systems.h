#pragma once

#include <string>
#include <vector>

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

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class LcmTrajectoryDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmTrajectoryDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                               std::string trajectory_name);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_input_port_);
  }

  void SetLineColor(drake::geometry::Rgba rgba) { rgba_ = rgba; }

  void SetNumSamples(int N) {
    DRAKE_DEMAND(N > 1);
    N_ = N;
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
  drake::geometry::Rgba rgba_ = drake::geometry::Rgba(0.1, 0.1, 0.1, 1.0);
  int N_ = 5;
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws the object pose through meshcat.
class LcmPoseDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmPoseDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                         const std::string& model_file,
                         const std::string& translation_trajectory_name,
                         const std::string& orientation_trajectory_name,
                         int num_poses = 5);

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
  const std::string translation_trajectory_name_;
  const std::string orientation_trajectory_name_;
  std::unique_ptr<multibody::MultiposeVisualizer> multipose_visualizer_;
  const int N_;
};

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class LcmForceDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmForceDrawer(const std::shared_ptr<drake::geometry::Meshcat>&,
                          std::string force_trajectory_name,
                          std::string actor_trajectory_name,
                          std::string lcs_force_trajectory_name);

  const drake::systems::InputPort<double>& get_input_port_actor_trajectory() const {
    return this->get_input_port(actor_trajectory_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_force_trajectory() const {
    return this->get_input_port(force_trajectory_input_port_);
  }

  void SetLineColor(drake::geometry::Rgba rgba) { rgba_ = rgba; }

  void SetNumSamples(int N) {
    DRAKE_DEMAND(N > 1);
    N_ = N;
  }

 private:
  drake::systems::EventStatus DrawForce(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;
  drake::systems::EventStatus DrawForces(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  drake::systems::InputPortIndex actor_trajectory_input_port_;
  drake::systems::InputPortIndex force_trajectory_input_port_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const std::string force_path_ = "c3_forces";
  const std::string actor_trajectory_name_;
  const std::string force_trajectory_name_;
  const std::string lcs_force_trajectory_name_;
  drake::geometry::Rgba rgba_ = drake::geometry::Rgba(0.1, 0.1, 0.1, 1.0);
  int N_ = 5;
  const double radius_ = 0.002;
  const double newtons_per_meter_ = 40;
};

}  // namespace systems
}  // namespace dairlib
