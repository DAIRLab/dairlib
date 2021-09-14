#pragma once

#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace goldilocks_models {

///
/// CurrentStanceFoot
///
class CurrentStanceFoot : public drake::systems::LeafSystem<double> {
 public:
  CurrentStanceFoot(const std::vector<int>& left_right_support_fsm_states);

  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(controller_signal_port_);
  }

 private:
  void GetStance(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* stance_foot) const;
  int controller_signal_port_;
  const std::vector<int>& left_right_support_fsm_states_;
  mutable bool start_with_right_stance_ = false;
};

///
/// PhaseInFirstMode
///

class PhaseInFirstMode : public drake::systems::LeafSystem<double> {
 public:
  PhaseInFirstMode(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      double stride_period);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(controller_signal_port_);
  }

 private:
  void CalcPhase(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* init_phase_output) const;

  // Port indices
  int state_port_;
  int controller_signal_port_;

  double stride_period_;
};

///
/// PlannerFinalPosition
///

class PlannerFinalPosition : public drake::systems::LeafSystem<double> {
 public:
  PlannerFinalPosition(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const Eigen::VectorXd& global_target_pos);
  PlannerFinalPosition(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const Eigen::VectorXd& const_step_length, int n_step);
  PlannerFinalPosition(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      double stride_period, int n_step);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_init_phase() const {
    return this->get_input_port(phase_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(controller_signal_port_);
  }

 private:
  PlannerFinalPosition(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      int high_level_command_mode);

  void CalcFinalPos(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* init_phase_output) const;

  // Port indices
  int state_port_;
  int phase_port_;
  int controller_signal_port_;

  int high_level_command_mode_;
  Eigen::VectorXd global_target_pos_;
  Eigen::VectorXd const_step_length_;
  int n_step_;
  double stride_period_;
};

///
/// InitialStateForPlanner
///
class InitialStateForPlanner : public drake::systems::LeafSystem<double> {
 public:
  InitialStateForPlanner(
      const drake::multibody::MultibodyPlant<double>& plant_feedback,
      const drake::multibody::MultibodyPlant<double>& plant_control, int n_step,
      double stride_period, bool feedback_is_spring_model);

  const drake::systems::InputPort<double>& get_input_port_stance_foot() const {
    return this->get_input_port(stance_foot_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_init_phase() const {
    return this->get_input_port(phase_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_and_lo_time()
      const {
    return this->get_input_port(controller_signal_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_adjusted_state()
      const {
    return this->get_output_port(adjusted_state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_adjustment() const {
    return this->get_output_port(adjustment_port_);
  }

 private:
  void CopyAdjustedState(const drake::systems::Context<double>& context,
                         systems::OutputVector<double>* output) const;
  void CopyAdjustment(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;

  drake::systems::EventStatus AdjustState(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  // Port indices
  int stance_foot_port_;
  int state_port_;
  int phase_port_;
  int controller_signal_port_;
  int adjusted_state_port_;
  int adjustment_port_;

  int adjusted_state_idx_;
  int quat_xyz_shift_idx_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  std::map<std::string, int> pos_map_w_spr_;
  std::map<std::string, int> pos_map_wo_spr_;
  std::map<std::string, int> vel_map_wo_spr_;

  int nq_;
  int nv_;

  // Parameters from MPC
  int n_step_;
  double stride_period_;

  // Stance foot height
  BodyPoint toe_mid_left_;
  BodyPoint toe_mid_right_;
  // It doesn't matter what initial value of prev_is_left_stance_ is, because
  // stance_foot_height_ in the beginning is always 0 (assuming the robot starts
  // when the feet are on the ground).
  mutable bool prev_is_left_stance_ = false;
  mutable double stance_foot_height_ = 0;
  double GetStanceFootHeight(
      const BodyPoint& stance_toe_mid,
      const drake::systems::Context<double>& context) const;

  // Testing
  const drake::multibody::MultibodyPlant<double>& plant_feedback_;
  const drake::multibody::MultibodyPlant<double>& plant_control_;
  bool feedback_is_spring_model_;

  // Zero toe joint velocity
  double window_ = 0.1;  // in seconds
  int idx_toe_leftdot_;
  int idx_toe_rightdot_;

  // IK
  drake::solvers::EqualityConstrainedQPSolver qp_solver_;
  std::vector<int> spring_pos_idx_list_w_spr_;
  std::vector<int> knee_ankle_pos_idx_list_;
  std::vector<int> knee_ankle_vel_idx_list_;
  std::vector<int> joint_vel_idx_list_left_;
  std::vector<int> joint_vel_idx_list_right_;
  std::unique_ptr<drake::systems::Context<double>> context_feedback_;
  std::unique_ptr<drake::systems::Context<double>> context_control_;
  BodyPoint toe_origin_left_;
  BodyPoint toe_origin_right_;
  double ik_feas_tol_ = 1e-4;  // 1e-2
  double ik_opt_tol_ = 1e-4;   // 1e-2
  void AdjustKneeAndAnklePos(const Eigen::VectorXd& x_w_spr,
                             const Eigen::Vector3d& left_foot_pos,
                             const Eigen::Vector3d& right_foot_pos,
                             const Eigen::VectorXd& x_init_original,
                             Eigen::VectorXd* x_init) const;
  void AdjustKneeAndAnkleVel(const Eigen::Vector3d& left_foot_vel,
                             const Eigen::Vector3d& right_foot_vel,
                             const Eigen::VectorXd& x_init_original,
                             Eigen::VectorXd* x_init) const;
  void ZeroOutStanceFootVel(bool is_left_stance, Eigen::VectorXd* x_init) const;
  void CheckAdjustemnt(const Eigen::VectorXd& x_w_spr,
                       const Eigen::VectorXd& x_original,
                       const Eigen::VectorXd& x_adjusted2,
                       const Eigen::Vector3d& left_foot_pos_w_spr,
                       const Eigen::Vector3d& right_foot_pos_w_spr,
                       const Eigen::Vector3d& left_foot_vel_w_spr,
                       const Eigen::Vector3d& right_foot_vel_w_spr,
                       bool is_left_stance) const;
};

}  // namespace goldilocks_models
}  // namespace dairlib
