#pragma once

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Creates a cubic spline interpolating between the most recent stance location
/// and a footstep target given by an input port

class SwingFootTargetTrajGen : public drake::systems::LeafSystem<double> {
 public:
  SwingFootTargetTrajGen(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::vector<int> left_right_support_fsm_states,
      std::vector<std::pair<const Eigen::Vector3d,
                            const drake::multibody::Frame<double>&>>
      left_right_foot,
      double mid_foot_height, double desired_final_foot_height,
      double desired_final_vertical_foot_velocity,
      bool relative_to_com=true);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(touchdown_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footstep_target()
  const {
    return this->get_input_port(footstep_target_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::trajectories::PiecewisePolynomial<double> CreateSplineForSwingFoot(
      double start_time_of_this_interval, double end_time_of_this_interval,
      const Eigen::Vector3d& init_swing_foot_pos,
      const Eigen::Vector3d& final_swing_foot_pos) const;

  void CalcTrajs(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* traj) const;

  int state_port_;
  int fsm_port_;
  int liftoff_time_port_;
  int touchdown_time_port_;
  int footstep_target_port_;

  int liftoff_swing_foot_pos_idx_;
  int prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  std::vector<int> left_right_support_fsm_states_;

  // Parameters
  double mid_foot_height_;
  double desired_final_foot_height_;
  double desired_final_vertical_foot_velocity_;
  bool relative_to_com_;

  // Maps
  std::map<int,
          std::pair<const Eigen::Vector3d,
                    const drake::multibody::Frame<double>&>> stance_foot_map_;
  std::map<int,
          std::pair<const Eigen::Vector3d,
                    const drake::multibody::Frame<double>&>> swing_foot_map_;

};

}  // namespace systems
}  // namespace dairlib
