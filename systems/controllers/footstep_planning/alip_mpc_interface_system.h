#pragma once

#include "alip_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/alip_traj_gen.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// Given an FSM signal and footsteps from an Alip MPC planner, this system
/// outputs swing foot trajectories and a height offset for use by the com traj
/// generator

struct SwingFootInterfaceSystemParams {
  std::vector<int> left_right_support_fsm_states;
  std::vector<alip_utils::PointOnFramed>left_right_foot;
  double com_height_;
  double mid_foot_height;
  double foot_height_offset_;
  double desired_final_foot_height;
  double desired_final_vertical_foot_velocity;
  bool relative_to_com = true;
};

class SwingFootInterfaceSystem : public drake::systems::LeafSystem<double> {
 public:
  SwingFootInterfaceSystem(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *context,
      const SwingFootInterfaceSystemParams& params);

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
  const drake::systems::OutputPort<double>& get_output_port_swing_foot_traj()
  const {
    return this->get_output_port(swing_foot_traj_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com_height_offset()
  const {
    return this->get_output_port(com_height_offset_output_port_);
  }

 private:

  bool is_single_support(int fsm_state) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double> &context,
      drake::systems::DiscreteValues<double> *discrete_state) const;

  drake::trajectories::PathParameterizedTrajectory<double>
  CreateSplineForSwingFoot(
      double start_time, double end_time, const Eigen::Vector3d &init_pos,
      const Eigen::Vector3d &final_pos) const;

  void CalcSwingTraj(const drake::systems::Context<double> &context,
                     drake::trajectories::Trajectory<double> *traj) const;

  void CopyComHeightOffset(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* com_height_offset) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex liftoff_time_port_;
  drake::systems::InputPortIndex touchdown_time_port_;
  drake::systems::InputPortIndex footstep_target_port_;

  drake::systems::OutputPortIndex swing_foot_traj_output_port_;
  drake::systems::OutputPortIndex com_height_offset_output_port_;

  drake::systems::DiscreteStateIndex liftoff_swing_foot_pos_idx_;
  drake::systems::DiscreteStateIndex prev_fsm_state_idx_;

  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *plant_context_;
  const drake::multibody::BodyFrame<double> &world_;

  std::vector<int> left_right_support_fsm_states_;

  // Parameters
  const double com_height_;
  const double mid_foot_height_;
  const double foot_height_offset_;
  const double desired_final_foot_height_;
  const double desired_final_vertical_foot_velocity_;
  const bool relative_to_com_;

  // Maps
  std::map<int, alip_utils::PointOnFramed> stance_foot_map_;
  std::map<int, alip_utils::PointOnFramed> swing_foot_map_;
};

class AlipMPCInterfaceSystem : public drake::systems::Diagram<double> {
 public:
  AlipMPCInterfaceSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      ALIPTrajGeneratorParams com_params,
      SwingFootInterfaceSystemParams swing_params);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(prev_liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(next_touchdown_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_footstep_target()
  const {
    return this->get_input_port(footstep_target_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com_traj() const {
    return this->get_output_port(com_traj_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_swing_foot_traj()
  const {
    return this->get_output_port(swing_traj_port_);
  }

 private:

  const drake::systems::InputPortIndex ExportSharedInput(
      drake::systems::DiagramBuilder<double>& builder,
      const drake::systems::InputPort<double>& p1,
      const drake::systems::InputPort<double>& p2, std::string name);

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex next_touchdown_time_port_;
  drake::systems::InputPortIndex prev_liftoff_time_port_;
  drake::systems::InputPortIndex footstep_target_port_;
  drake::systems::OutputPortIndex com_traj_port_;
  drake::systems::OutputPortIndex swing_traj_port_;

};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
