#pragma once

#include "alip_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/footstep_planning/swing_foot_traj_solver.h"

#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// Given an FSM signal and footsteps from an MPC footstep planner, this system
/// outputs swing foot trajectories. The footstep target and the swing foot
/// trajectory should both be interpreted to be in the stance frame. That is the
/// frame located at the bottom center of the stance foot, rotated about it's z
/// axis to align with the floating base yaw frame.

struct SwingFootTrajectoryGeneratorParams {
  std::vector<int> left_right_support_fsm_states;
  std::vector<alip_utils::PointOnFramed> left_right_foot;
  std::vector<int> post_left_post_right_fsm_states;
  double mid_foot_height;
  double desired_final_foot_height;
  double desired_final_vertical_foot_velocity;
  double retraction_dist = 0.07;
  bool used_with_sim=false;
};

class SwingFootTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  SwingFootTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *context,
      const SwingFootTrajectoryGeneratorParams& params);

  const drake::systems::InputPort<double> &get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double> &get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(touchdown_time_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_footstep_target()
  const {
    return this->get_input_port(footstep_target_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot_traj()
  const {
    return this->get_output_port(swing_foot_traj_output_port_);
  }

  drake::systems::DiscreteStateIndex liftoff_swing_foot_pos_state_idx()
  const {
    return liftoff_swing_foot_pos_idx_;
  }

  void MakeDrivenByStandaloneSimulator() {
    DeclarePerStepUnrestrictedUpdateEvent(
        &SwingFootTrajectoryGenerator::UnrestrictedUpdate);
  }

 private:

  bool is_single_support(int fsm_state) const;

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::State<double> *state) const;


  void CalcSwingTraj(const drake::systems::Context<double> &context,
                     drake::trajectories::Trajectory<double> *traj) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex liftoff_time_port_;
  drake::systems::InputPortIndex touchdown_time_port_;
  drake::systems::InputPortIndex footstep_target_port_;

  drake::systems::OutputPortIndex swing_foot_traj_output_port_;

  drake::systems::AbstractStateIndex prev_spline_idx_;
  drake::systems::DiscreteStateIndex prev_time_idx_;
  drake::systems::DiscreteStateIndex liftoff_swing_foot_pos_idx_;
  drake::systems::DiscreteStateIndex prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  const drake::multibody::BodyFrame<double> &world_;

  std::vector<int> left_right_support_fsm_states_;

  // Parameters
  const double retraction_dist_;
  const double mid_foot_height_;
  const double desired_final_foot_height_;
  const double desired_final_vertical_foot_velocity_;
  const bool used_with_sim_;

  // Maps
  std::map<int, alip_utils::PointOnFramed> stance_foot_map_;
  std::map<int, alip_utils::PointOnFramed> swing_foot_map_;

  // Solver
  mutable SwingFootTrajSolver foot_traj_solver_{};
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib