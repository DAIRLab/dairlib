#pragma once

#include <memory>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "dairlib/lcmt_franka_hand_target_position.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "parameter_headers/assembly_c3_options.h"
#include "parameter_headers/target_poses.h"
#include "solvers/c3_plus.h"
#include "solvers/lcs_factory.h"

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
      const AssemblyC3Options& c3_options,
      const TargetPosesParams& target_poses_params, bool verbose = false);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
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

 private:
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Helper functions for different phases
  void GenerateMoveToTargetTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                      double t_context, LcmTrajectory* traj,
                                      int target_index) const;

  /// Check if target is reached and return true if so
  bool IsTargetReached(const Eigen::VectorXd& x_lcs_curr,
                       int target_index) const;

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
                             LcmTrajectory* planned_keypoints_traj) const;

  /// Output port function
  void OutputTrajExecute(const drake::systems::Context<double>& context,
                         dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputTrajPlannedKeypoints(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputGripperPosCommand(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_franka_hand_target_position* output) const;

  // Input/output port indices
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::OutputPortIndex traj_execute_port_;
  drake::systems::OutputPortIndex traj_planned_keypoints_port_;
  drake::systems::OutputPortIndex gripper_pos_command_port_;

  // Plant references
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;

  // C3 options
  AssemblyC3Options assembly_c3_options_;
  solvers::ContactModel contact_model_;

  // System dimensions
  int n_q_;
  int n_v_;
  int n_u_;
  int n_x_;
  int n_lambda_;
  int N_;
  double dt_;

  // Cost matrices
  mutable std::vector<Eigen::MatrixXd> Q_;
  mutable std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> U_;

  // Phase management
  mutable AssemblyPhase current_phase_ = AssemblyPhase::kMoveToTarget;
  drake::systems::DiscreteStateIndex phase_index_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  drake::systems::DiscreteStateIndex current_target_index_;
  drake::systems::DiscreteStateIndex
      target_reached_time_index_;  // Time when target was first reached (for
                                   // dwell)

  // MPC solver
  mutable std::shared_ptr<solvers::C3Plus> c3_mpc_;

  // Execution trajectory
  mutable LcmTrajectory execution_lcm_traj_;
  mutable double gripper_pos_command_ = 0.025;

  // Planned keypoints trajectory
  mutable LcmTrajectory planned_keypoints_lcm_traj_;

  mutable std::vector<TargetPose> target_poses_;

  bool verbose_;
};

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
