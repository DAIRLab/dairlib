#pragma once

#include "alip_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// Creates a cubic spline interpolating between the most recent stance location
/// and a footstep target given by an input port

class SwingFootAndComTrajGen : public drake::systems::LeafSystem<double> {
 public:
  SwingFootAndComTrajGen(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *context,
      std::vector<int> left_right_support_fsm_states,
      std::vector<alip_utils::PointOnFramed> left_right_foot,
      double mid_foot_height, double desired_final_foot_height,
      double desired_final_vertical_foot_velocity,
      bool relative_to_com = true);

  const drake::systems::InputPort<double> &get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(touchdown_time_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_footstep_target()
  const {
    return this->get_input_port(footstep_target_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  drake::trajectories::PathParameterizedTrajectory<double>
  CreateSplineForSwingFoot(
      double start_time, double end_time, const Eigen::Vector3d &init_pos,
      const Eigen::Vector3d &final_pos) const;

  void CalcTrajs(const drake::systems::Context<double> &context,
                 drake::trajectories::Trajectory<double> *traj) const;

  int state_port_;
  int fsm_port_;
  int liftoff_time_port_;
  int touchdown_time_port_;
  int footstep_target_port_;

  int liftoff_swing_foot_pos_idx_;
  int prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *context_;
  const drake::multibody::BodyFrame<double> &world_;

  std::vector<int> left_right_support_fsm_states_;

  // Parameters
  double mid_foot_height_;
  double desired_final_foot_height_;
  double desired_final_vertical_foot_velocity_;
  bool relative_to_com_;

  // Maps
  std::map<int, alip_utils::PointOnFramed> stance_foot_map_;
  std::map<int, alip_utils::PointOnFramed> swing_foot_map_;

};

}  // namesoace controllers
}  // namespace systems
}  // namespace dairlib
