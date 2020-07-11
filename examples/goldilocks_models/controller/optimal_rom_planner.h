#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

class OptimalRomPlanner : public drake::systems::LeafSystem<double> {
 public:
  OptimalRomPlanner(
      const drake::multibody::MultibodyPlant<double>& plant,
      double desired_com_height, const std::vector<int>& unordered_fsm_states,
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

 private:
  // Discrete update calculates and stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  // Port indices
  int state_port_;
  int fsm_port_;

  // Discrete state indices
  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;

  double desired_com_height_;

  std::vector<int> unordered_fsm_states_;
  std::vector<double> unordered_state_durations_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<std::vector<std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state_;

  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
