#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

/// This class creates predicted center of mass (COM) trajectory of a bipedal
/// robot.
/// The trajectories in horizontal directions (x and y axes) are predicted, and
/// the traj in the vertical direction (z axis) starts/ends at the
/// current/desired height.

/// Constructor inputs:
///  @param tree, rigid body tree
///  @param desired_com_height, desired COM height
///  @param unordered_fsm_states, vector of fsm states
///  @param unordered_state_durations, duration of each state in
///         unordered_fsm_states
///  @param body_indices, body indices of tree for calculating the stance foot
///         position (of each state in unordered_fsm_states). If there are two
///         or more indices, we get the average of the positions.
///  @param pts_on_bodies, position of the points on the bodies for calculating
///         the stance foot position (of each state in unordered_fsm_states).
/// The last four parameters must have the same size.

class LIPMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  LIPMTrajGenerator(
      const RigidBodyTree<double>& tree, double desired_com_height,
      const std::vector<int>& unordered_fsm_states,
      const std::vector<double>& unordered_state_durations,
      const std::vector<std::vector<int>>& body_indices,
      const std::vector<std::vector<Eigen::Vector3d>>& pts_on_bodies);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  // Discrete update calculates and stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  int state_port_;
  int fsm_port_;

  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  const RigidBodyTree<double>& tree_;

  double desired_com_height_;

  bool is_quaternion_;

  std::vector<int> unordered_fsm_states_;
  std::vector<double> unordered_state_durations_;
  std::vector<std::vector<int>> body_indices_;
  std::vector<std::vector<Eigen::Vector3d>> pts_on_bodies_;
};

}  // namespace systems
}  // namespace dairlib
