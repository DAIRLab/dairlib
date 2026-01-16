#pragma once

#include <memory>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "dairlib/lcmt_c3_forces.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "parameter_headers/assembly_c3_options.h"
#include "parameter_headers/osc_target_poses.h"
#include "parameter_headers/round_belt_controller_params.h"
#include "solvers/c3_output.h"
#include "solvers/c3_plus.h"
#include "solvers/lcs_factory.h"

#include "drake/lcmt_schunk_wsg_command.hpp"

namespace dairlib {
namespace examples {
namespace magna {

enum class AssemblyPhase { kMoveToTarget, kMPC };

class AssemblyController : public drake::systems::LeafSystem<double> {
 public:
  AssemblyController(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<
          std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
          contact_geoms,
      const RoundBeltControllerParams& round_belt_controller_params,
      bool verbose = false);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_task_relevant_keypoints() const {
    return this->get_input_port(task_relevant_keypoints_input_port_);
  }

  // Output ports
  const drake::systems::OutputPort<double>& get_output_port_traj_execute()
      const {
    return this->get_output_port(traj_execute_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_traj_planned_keypoints() const {
    return this->get_output_port(traj_planned_keypoints_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_gripper_pos_command() const {
    return this->get_output_port(gripper_pos_command_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_forces() const {
    return this->get_output_port(c3_forces_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_current_target_lcs_state() const {
    return this->get_output_port(current_target_lcs_state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates()
      const {
    return this->get_output_port(c3_intermediates_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_solution()
      const {
    return this->get_output_port(c3_solution_port_);
  }

 private:
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Helper methods for state management
  struct DiscreteStateData {
    double plan_start_time;
    AssemblyPhase current_phase;
    int current_target_idx;
    double target_reached_time;
    int mpc_current_target_idx;
  };

  DiscreteStateData ExtractDiscreteState(
      const drake::systems::DiscreteValues<double>& discrete_state) const;

  void UpdateDiscreteState(
      const DiscreteStateData& data,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Check if we need to advance to the next target
  bool ShouldAdvanceToNextTarget(const Eigen::VectorXd& x_lcs_curr,
                                 double current_time, int target_index,
                                 double target_reached_time) const;

  /// Helper functions for different phases
  void GenerateMoveToTargetTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                      double t_context, LcmTrajectory* traj,
                                      int target_index) const;

  /// Check if target is reached and return true if so (for kMoveToTarget phase)
  bool IsTargetReached(const Eigen::VectorXd& x_lcs_curr,
                       int target_index) const;

  /// Check if MPC target is reached (for kMPC phase)
  bool IsMpcTargetReached(const Eigen::VectorXd& x_lcs_curr,
                          const Eigen::VectorXd& x_lcs_des) const;

  /// Helper function to add position, orientation, and force trajectories to
  /// LcmTrajectory
  void AddEETrajectoriesToLcm(const Eigen::MatrixXd& position_knots,
                              const Eigen::MatrixXd& orientation_knots,
                              const Eigen::MatrixXd& force_knots,
                              const Eigen::VectorXd& timestamps,
                              LcmTrajectory* traj) const;
  void GenerateGripperControlTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                        double t_context, LcmTrajectory* traj,
                                        double gripper_pos_command,
                                        double dwell_time) const;
  void GenerateMPCTrajectory(const Eigen::VectorXd& x_lcs_curr,
                             const Eigen::VectorXd& x_lcs_des, double t_context,
                             LcmTrajectory* traj,
                             LcmTrajectory* planned_keypoints_traj,
                             DiscreteStateData* state_data) const;

  void ConvertForcesToWorldFrame(
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
          resolved_contact_pairs,
      const Eigen::VectorXd& lcs_forces) const;

  /// Output port function
  void OutputTrajExecute(const drake::systems::Context<double>& context,
                         dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputTrajPlannedKeypoints(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputGripperPosCommand(const drake::systems::Context<double>& context,
                               drake::lcmt_schunk_wsg_command* output) const;
  void OutputC3Forces(const drake::systems::Context<double>& context,
                      dairlib::lcmt_c3_forces* output) const;
  void OutputCurrentTargetLcsState(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  void OutputC3Intermediates(const drake::systems::Context<double>& context,
                             C3Output::C3Intermediates* c3_intermediates) const;
  void OutputC3Solution(const drake::systems::Context<double>& context,
                        C3Output::C3Solution* c3_solution) const;

  // Input/output port indices
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex task_relevant_keypoints_input_port_;
  drake::systems::OutputPortIndex traj_execute_port_;
  drake::systems::OutputPortIndex traj_planned_keypoints_port_;
  drake::systems::OutputPortIndex gripper_pos_command_port_;
  drake::systems::OutputPortIndex c3_forces_port_;
  drake::systems::OutputPortIndex current_target_lcs_state_port_;
  drake::systems::OutputPortIndex c3_intermediates_port_;
  drake::systems::OutputPortIndex c3_solution_port_;

  // Plant references
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;

  mutable RoundBeltControllerParams round_belt_controller_params_;

  // C3 options
  AssemblyC3Options assembly_c3_options_;
  solvers::ContactModel contact_model_;

  // System dimensions
  int n_q_;
  int n_v_;
  int n_u_;
  int n_x_;
  int n_lambda_;
  int n_z_;
  int N_;
  double dt_;

  // Cost matrices
  mutable std::vector<Eigen::MatrixXd> Q_;
  mutable std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> U_;

  // Phase management
  drake::systems::DiscreteStateIndex phase_index_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  drake::systems::DiscreteStateIndex current_target_index_;
  drake::systems::DiscreteStateIndex
      target_reached_time_index_;  // Time when target was first reached (for
                                   // dwell)
  // MPC target management
  drake::systems::DiscreteStateIndex mpc_current_target_index_;
  mutable std::vector<Eigen::VectorXd> mpc_target_lcs_states_;

  // MPC solver
  mutable std::shared_ptr<solvers::C3Plus> c3_mpc_;

  // Execution trajectory
  mutable LcmTrajectory execution_lcm_traj_;
  mutable double gripper_pos_command_ = 0.025;

  // Planned keypoints trajectory
  mutable LcmTrajectory planned_keypoints_lcm_traj_;

  // C3 forces output
  mutable dairlib::lcmt_c3_forces c3_forces_output_;

  mutable std::vector<SingleOSCTargetPose> osc_target_poses_;

  bool verbose_;

  mutable bool is_solve_succeeded_ = true;
  mutable bool mpc_reached_target_ = false;

  mutable bool track_ee_force_ = true;

  // Task-relevant keypoints storage
  mutable std::vector<std::vector<double>> task_relevant_keypoints_;
  mutable double task_relevant_keypoints_avg_z_ = 0.0;
  mutable double target_ee_height_ =
      0.0;  // difference between the average z and the height of the small
            // pulley's groove
  double groove_height_ = 0.038;
};

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
