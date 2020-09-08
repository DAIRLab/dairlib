#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
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
///  @param plant, the MultibodyPlant
///  @param desired_com_height, desired COM height
///  @param unordered_fsm_states, vector of fsm states
///  @param unordered_state_durations, duration of each state in
///         unordered_fsm_states
///  @param contact_points_in_each_state, <position of the points on the bodies,
///         body frame> pairs of plant for calculating the stance foot
///         position (of each state in unordered_fsm_states). If there are two
///         or more pairs, we get the average of the positions.
/// The last three parameters must have the same size.

class LIPMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  LIPMTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, double desired_com_height,
      const std::vector<int>& unordered_fsm_states,
      const std::vector<double>& unordered_state_durations,
      const std::vector<std::vector<std::pair<
          const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
          contact_points_in_each_state);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
      const {
    return this->get_input_port(fsm_switch_time_port_);
  }

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  // Port indices
  int state_port_;
  int fsm_port_;
  int fsm_switch_time_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  double desired_com_height_;
  std::vector<int> unordered_fsm_states_;

  std::vector<double> unordered_state_durations_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<std::vector<std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state_;
  const drake::multibody::BodyFrame<double>& world_;
};

}  // namespace systems
}  // namespace dairlib
