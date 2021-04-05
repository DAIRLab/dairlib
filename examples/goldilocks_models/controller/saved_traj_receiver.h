#pragma once

#include "examples/goldilocks_models/reduced_order_models.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

// One use case of ExponentialPlusPiecewisePolynomial: if we want to use a guard
// that merge optimal ROM traj with LIPM traj.

class SavedTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  SavedTrajReceiver(
      const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_control,
      drake::systems::Context<double>* context_feedback,
      const std::vector<BodyPoint>& left_right_foot,
      std::vector<int> left_right_support_fsm_states, bool both_pos_vel_in_traj,
      double single_support_duration, double double_support_duration);

  const drake::systems::InputPort<double>& get_input_port_lcm_traj() const {
    return this->get_input_port(saved_traj_lcm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_rom() const {
    return this->get_output_port(rom_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot() const {
    return this->get_output_port(swing_foot_traj_port_);
  }

 private:
  void CalcRomTraj(const drake::systems::Context<double>& context,
                   drake::trajectories::Trajectory<double>* traj) const;
  void CalcSwingFootTraj(const drake::systems::Context<double>& context,
                         drake::trajectories::Trajectory<double>* traj) const;
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int saved_traj_lcm_port_;
  int state_port_;
  int fsm_port_;

  int rom_traj_port_;
  int swing_foot_traj_port_;

  int liftoff_swing_foot_pos_idx_;
  mutable double prev_fsm_state_ = -1;
  mutable double lift_off_time_ = 0;

  int ny_;
  const drake::multibody::MultibodyPlant<double>& plant_feedback_;
  const drake::multibody::MultibodyPlant<double>& plant_control_;
  drake::systems::Context<double>* context_feedback_;
  std::unique_ptr<drake::systems::Context<double>> context_control_;
  const std::vector<BodyPoint>& left_right_foot_;
  std::vector<int> left_right_support_fsm_states_;

  int nq_;
  int nv_;
  int nx_;
  bool both_pos_vel_in_traj_;

  std::map<int, BodyPoint> swing_foot_map_;

  // hacks
  double single_support_duration_;
  double double_support_duration_;
};

// We have IKTrajReceiver beside SavedTrajReceiver, because it also extracts the
// rows of the trajectory matrix that we want to track.
class IKTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  IKTrajReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                 const std::vector<std::string>& ordered_pos_names);

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  int saved_traj_lcm_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<int> ordered_indices_;

  int nq_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
