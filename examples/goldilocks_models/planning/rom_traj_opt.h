#pragma once

#include <tuple>
#include <vector>

#include <memory.h>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

using drake::solvers::Binding;
using drake::solvers::Cost;

namespace dairlib {
namespace goldilocks_models {

/// Reduced order model
/// z = [y; ydot]
/// yddot = theta_yddot * phi_yddot + B * tau

/// Notes that we are currently constructing cubic splines for the state. This
/// is different form the model optimization where we construct cubic splines
/// for the position.

// Modified from HybridDircon class
class RomTrajOpt
    : public drake::systems::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RomTrajOpt)

  RomTrajOpt(const std::vector<int>& num_time_samples, const Eigen::MatrixXd& Q,
             const Eigen::MatrixXd& R, const ReducedOrderModel& rom,
             const drake::multibody::MultibodyPlant<double>& plant,
             const StateMirror& state_mirror,
             const std::vector<BodyPoint>& left_contacts,
             const std::vector<BodyPoint>& right_contacts,
             const BodyPoint& left_origin, const BodyPoint& right_origin,
             const std::vector<std::tuple<std::string, double, double>>&
                 fom_joint_name_lb_ub,
             const Eigen::VectorXd& x_init,
             const Eigen::VectorXd& rom_state_init,
             const std::vector<double>& max_swing_distance,
             bool start_with_left_stance, bool zero_touchdown_impact,
             const std::set<int>& relax_index, const PlannerSetting& param,
             bool initialize_with_rom_state, bool print_status = true);
  ~RomTrajOpt() override {}

  void AddConstraintAndCostForLastFootStep(
      double w_predict_lipm_v, const Eigen::VectorXd& des_predicted_xy_vel,
      double stride_period);

  void AddCascadedLipmMPC(double w_predict_lipm_p, double w_predict_lipm_v,
                          const std::vector<Eigen::VectorXd>& des_xy_pos,
                          const std::vector<Eigen::VectorXd>& des_xy_vel,
                          int n_step_lipm, double stride_period,
                          double max_step_length, double min_step_width);

  void AddTimeStepConstraint(std::vector<double> minimum_timestep,
                             std::vector<double> maximum_timestep,
                             bool fix_duration, bool equalize_timestep_size,
                             double first_mode_duration,
                             double remaining_mode_duration);

  /// Returns a vector of matrices containing the state and derivative values at
  /// each breakpoint at the solution for each mode of the trajectory.
  void GetStateAndDerivativeSamples(
      const drake::solvers::MathematicalProgramResult& result,
      std::vector<Eigen::MatrixXd>* state_samples,
      std::vector<Eigen::MatrixXd>* derivative_samples,
      std::vector<Eigen::VectorXd>* state_breaks) const;
  /// Returns a vector of matrices containing the state values at
  /// each breakpoint at the solution for each mode of the trajectory.
  void GetStateSamples(const drake::solvers::MathematicalProgramResult& result,
                       std::vector<Eigen::MatrixXd>* state_samples,
                       std::vector<Eigen::VectorXd>* state_breaks) const;

  /// Get the input trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const override;

  /// Get the state trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const drake::solvers::MathematicalProgramResult& result) const override;

  const drake::solvers::VectorXDecisionVariable& z_post_impact_vars() const {
    return z_post_impact_vars_;
  }

  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  z_post_impact_vars_by_mode(int mode) const;
  drake::solvers::VectorXDecisionVariable x0_vars_by_mode(int mode) const;
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  xf_vars_by_mode(int mode) const;

  /// Get the state decision variables given a mode and a time_index
  /// (time_index is w.r.t that particular mode). This will use the
  ///  z_post_impact_vars_ if needed. Otherwise, it just returns the standard
  /// x_vars element
  drake::solvers::VectorXDecisionVariable state_vars_by_mode(
      int mode, int time_index) const;

  drake::solvers::VectorXDecisionVariable impulse_vars(int mode) const;

  drake::solvers::VectorXDecisionVariable touchdown_foot_pos_vars(
      int mode) const;

  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  x_lipm_vars_by_idx(int idx) const;
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  u_lipm_vars_by_idx(int idx) const;

  drake::VectorX<drake::symbolic::Expression> SubstitutePlaceholderVariables(
      const drake::VectorX<drake::symbolic::Expression>& f,
      int interval_index) const;

  Eigen::VectorXd GetTimeStepSolution(
      const drake::solvers::MathematicalProgramResult& result) const {
    return result.GetSolution(h_vars());
  }

  int num_knots() const { return this->N(); }
  int num_modes() const { return num_modes_; }
  const std::vector<int>& mode_lengths() const { return mode_lengths_; }
  const std::vector<int>& mode_start() const { return mode_start_; }

  int n_lambda() const { return n_lambda_; }
  int n_x_FOM() const { return n_x_; }
  bool start_with_left_stance() const { return start_with_left_stance_; }

  const drake::solvers::VectorXDecisionVariable& h_vars() const {
    return MultipleShooting::h_vars();
  };

  const ReducedOrderModel& reduced_order_model() const { return rom_; }

  // Constraint scaling
  void SetScalingForLIPM();
  std::unordered_map<int, double> fom_discrete_dyn_constraint_scaling_;
  std::unordered_map<int, double> fom_guard_constraint_scaling_;
  std::unordered_map<int, double> fom_stance_ft_pos_constraint_scaling_;
  std::unordered_map<int, double> fom_stance_ft_vel_constraint_scaling_;
  std::unordered_map<int, double> rom_dyn_constraint_scaling_;
  std::unordered_map<int, double> rom_fom_mapping_constraint_scaling_;

  // Cost bindings
  std::vector<Binding<Cost>> rom_state_cost_bindings_;
  std::vector<Binding<Cost>> rom_input_cost_bindings_;
  std::vector<Binding<Cost>> rom_regularization_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_quat_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_xy_pos_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_z_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_joint_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_vel_cost_bindings_;
  std::vector<Binding<Cost>> fom_reg_xy_vel_cost_bindings_;
  std::vector<Binding<Cost>> x0_relax_cost_bindings_;
  std::vector<Binding<Cost>> v0_relax_cost_bindings_;
  std::vector<Binding<Cost>> init_rom_relax_cost_bindings_;
  std::vector<Binding<Cost>> lambda_cost_bindings_;
  std::vector<Binding<Cost>> predict_lipm_p_bindings_;
  std::vector<Binding<Cost>> predict_lipm_v_bindings_;

  // Decision variables
  const drake::solvers::VectorXDecisionVariable z_post_impact_vars_;
  const drake::solvers::VectorXDecisionVariable x0_var_;
  const drake::solvers::VectorXDecisionVariable xf_vars_;
  const drake::solvers::VectorXDecisionVariable v_post_impact_vars_;
  const drake::solvers::VectorXDecisionVariable impulse_vars_;
  drake::solvers::VectorXDecisionVariable touchdown_foot_pos_vars_;
  drake::solvers::VectorXDecisionVariable eps_rom_var_;
  drake::solvers::VectorXDecisionVariable predicted_com_vel_var_;
  drake::solvers::VectorXDecisionVariable x_lipm_vars_;
  drake::solvers::VectorXDecisionVariable u_lipm_vars_;

 protected:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  void DoAddRunningCost(const drake::symbolic::Expression& g,
                        std::vector<Binding<Cost>>* bindings);

  const int num_modes_;
  const std::vector<int> mode_lengths_;
  std::vector<int> mode_start_;
  const int n_y_;
  const int n_z_;
  const int n_q_;
  const int n_v_;
  const int n_x_;
  const int n_lambda_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const ReducedOrderModel& rom_;
  bool start_with_left_stance_;
  const BodyPoint& left_origin_;
  const BodyPoint& right_origin_;

  void PrintStatus(const std::string& msg) const {
    if (print_status_) std::cout << msg << std::endl;
  };
  bool print_status_;
};

class RomTrajOptCassie : public RomTrajOpt {
 public:
  RomTrajOptCassie(const std::vector<int>& num_time_samples,
                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                   const ReducedOrderModel& rom,
                   const drake::multibody::MultibodyPlant<double>& plant,
                   const StateMirror& state_mirror,
                   const std::vector<BodyPoint>& left_contacts,
                   const std::vector<BodyPoint>& right_contacts,
                   const BodyPoint& left_origin, const BodyPoint& right_origin,
                   const std::vector<std::tuple<std::string, double, double>>&
                       fom_joint_name_lb_ub,
                   const Eigen::VectorXd& x_init,
                   const Eigen::VectorXd& rom_state_init,
                   const std::vector<double>& max_swing_distance,
                   bool start_with_left_stance, bool zero_touchdown_impact,
                   const std::set<int>& relax_index,
                   const PlannerSetting& param, bool initialize_with_rom_state,
                   bool print_status = true);

  void AddFomRegularizationCost(const std::vector<Eigen::VectorXd>& reg_x_FOM,
                                double w_reg_quat, double w_reg_xy,
                                double w_reg_z, double w_reg_joints,
                                double w_reg_hip_yaw, double w_reg_xy_vel,
                                double w_reg_vel);
  // Testing -- AddRomRegularizationCost is SetAllInitialGuess except we replace
  // setting guess with setting cost
  void AddRomRegularizationCost(const Eigen::VectorXd& h_guess,
                                const Eigen::MatrixXd& y_guess,
                                const Eigen::MatrixXd& dy_guess,
                                const Eigen::MatrixXd& tau_guess,
                                int fisrt_mode_phase_index, double w_reg);

  void SetHeuristicInitialGuess(const PlannerSetting& param,
                                const Eigen::VectorXd& h_guess,
                                const Eigen::MatrixXd& y_guess,
                                const Eigen::MatrixXd& dy_guess,
                                const Eigen::MatrixXd& tau_guess,
                                const std::vector<Eigen::VectorXd>& reg_x_FOM,
                                int fisrt_mode_phase_index,
                                int starting_mode_index);
  void SetHeuristicInitialGuessForCascadedLipm(
      const PlannerSetting& param,
      const std::vector<Eigen::VectorXd>& des_xy_pos,
      const std::vector<Eigen::VectorXd>& des_xy_vel);

  drake::VectorX<double> quat_identity_;
};

class RomTrajOptFiveLinkRobot : public RomTrajOpt {
 public:
  RomTrajOptFiveLinkRobot(
      const std::vector<int>& num_time_samples, const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R, const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant,
      const StateMirror& state_mirror,
      const std::vector<BodyPoint>& left_contacts,
      const std::vector<BodyPoint>& right_contacts,
      const std::vector<std::tuple<std::string, double, double>>&
          fom_joint_name_lb_ub,
      Eigen::VectorXd x_init, bool start_with_left_stance,
      bool zero_touchdown_impact);

  void AddRegularizationCost(const Eigen::VectorXd& final_position,
                             const Eigen::VectorXd& x_guess_left_in_front,
                             const Eigen::VectorXd& x_guess_right_in_front,
                             bool straight_leg_cost);

  void SetAllInitialGuess(const Eigen::VectorXd& h_guess,
                          const Eigen::MatrixXd& r_guess,
                          const Eigen::MatrixXd& dr_guess,
                          const Eigen::MatrixXd& tau_guess,
                          const Eigen::VectorXd& x_guess_left_in_front,
                          const Eigen::VectorXd& x_guess_right_in_front,
                          const Eigen::VectorXd& final_position);
};

}  // namespace goldilocks_models
}  // namespace dairlib
