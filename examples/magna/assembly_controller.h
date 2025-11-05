#pragma once

#include <memory>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_franka_hand_target_position.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3_options.h"
#include "solvers/c3_plus.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace magna {

enum class AssemblyPhase {
  kPreGraspingMotion,
  kClosingGripper,
  kMovingUpAndLeft,
  kMovingDown,
  kMPC
};

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
      const C3Options& c3_options);

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
  get_output_port_gripper_pos_command() const {
    return this->get_output_port(gripper_pos_command_port_);
  }

 private:
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Helper functions for different phases
  void GeneratePreGraspingTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                   double t_context, LcmTrajectory* traj) const;
  void GenerateMovingUpAndLeftTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                         double t_context,
                                         LcmTrajectory* traj) const;
  void GenerateMovingDownTrajectory(const Eigen::VectorXd& x_lcs_curr,
                                    double t_context,
                                    LcmTrajectory* traj) const;
  //   void GenerateMPCTrajectory(
  //       const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des,
  //       double t_context, LcmTrajectory* traj) const;

  /// Output port function
  void OutputTrajExecute(const drake::systems::Context<double>& context,
                         dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputGripperPosCommand(const drake::systems::Context<double>& context,
                               dairlib::lcmt_franka_hand_target_position* output) const;

  // Input/output port indices
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::OutputPortIndex traj_execute_port_;
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
  C3Options c3_options_;
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
  mutable AssemblyPhase current_phase_ = AssemblyPhase::kPreGraspingMotion;
  drake::systems::DiscreteStateIndex phase_index_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;

  // MPC solver
  mutable std::shared_ptr<solvers::C3Plus> c3_mpc_;

  // Execution trajectory
  mutable LcmTrajectory execution_lcm_traj_;
  mutable double gripper_pos_command_ = 0.0;
};

}  // namespace magna
}  // namespace dairlib
