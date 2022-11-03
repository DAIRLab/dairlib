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

/// `HybridRomTrajOpt` differs from `RomTrajOpt` in that it does not contain the
/// full robot configuration/state in the planning problem.
/// This class is currently specific to physical interpretable model with
///     y = COM pos rt stance foot

/// Reduced order model
/// z = [y; ydot]
/// yddot = theta_yddot * phi_yddot + B * tau

/// Assumptions:
/// 1. we assume positions are relative to initial pelvis orientation (this
///  means that we cannot take too many steps when turning, because the step
///  length and feet collision constriant are expressed rt the inital pelvis
///  frame)

/// Notes that we are currently constructing cubic splines for the state. This
/// is different form the model optimization where we construct cubic splines
/// for the position.

/// Misc: We are currently using the same ROM continuous-time input for pre/post
/// impacts

// Modified from RomTrajOpt class
class HybridRomTrajOpt
    : public drake::systems::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridRomTrajOpt)

  HybridRomTrajOpt(
      const std::vector<int>& num_time_samples, const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R, const ReducedOrderModel& rom,
      const Eigen::VectorXd& init_rom_state,
      const std::vector<double>& max_swing_distance,
      bool start_with_left_stance, bool zero_touchdown_impact,
      const std::set<int>& relax_index, const PlannerSetting& param,
      const std::vector<int>& num_time_samples_ds = {},
      bool start_in_double_support_phase = false,
      const std::set<int>& idx_constant_rom_vel_during_double_support = {},
      bool print_status = true);
  ~HybridRomTrajOpt() override {}

  void AddConstraintAndCostForLastFootStep(
      double w_predict_lipm_v, const Eigen::VectorXd& des_predicted_xy_vel,
      double stride_period, double com_height);

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

  /// Get the state decision variables given a mode and a time_index
  /// (time_index is w.r.t that particular mode). This will use the
  ///  z_post_impact_vars_ if needed. Otherwise, it just returns the standard
  /// x_vars element
  drake::solvers::VectorXDecisionVariable state_vars_by_mode(
      int mode, int time_index) const;

  drake::solvers::VectorXDecisionVariable
  discrete_swing_foot_pos_rt_stance_foot_x_vars(int mode) const;
  drake::solvers::VectorXDecisionVariable
  discrete_swing_foot_pos_rt_stance_foot_y_vars(int mode) const;
  drake::solvers::VectorXDecisionVariable
  y_end_of_last_mode_rt_init_stance_foot_var() const;

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

  bool start_with_left_stance() const { return start_with_left_stance_; }

  const drake::solvers::VectorXDecisionVariable& h_vars() const {
    return MultipleShooting::h_vars();
  };

  const ReducedOrderModel& reduced_order_model() const { return rom_; }

  // Constraint scaling
  void SetScalingForLIPM();
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
  drake::solvers::VectorXDecisionVariable
      discrete_swing_foot_pos_rt_stance_foot_x_vars_;
  drake::solvers::VectorXDecisionVariable
      discrete_swing_foot_pos_rt_stance_foot_y_vars_;
  drake::solvers::VectorXDecisionVariable
      y_end_of_last_mode_rt_init_stance_foot_var_;
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
  const ReducedOrderModel& rom_;
  bool start_with_left_stance_;

  std::set<int> empty_idx_set_ = {};
  std::vector<int> empty_idx_vec_ = {};

  // Double support phase related
  const std::vector<int>& num_time_samples_ds_;
  bool use_double_support_mode_in_planner_;
  bool start_in_double_support_phase_;
  bool constant_rom_vel_during_double_support_;
  double single_support_duration_;
  double double_support_duration_;

  void PrintStatus(const std::string& msg) const {
    if (print_status_) std::cout << msg << std::endl;
  };
  bool print_status_;
};

class HybridRomTrajOptCassie : public HybridRomTrajOpt {
 public:
  HybridRomTrajOptCassie(
      const std::vector<int>& num_time_samples, const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R, const ReducedOrderModel& rom,
      const Eigen::VectorXd& init_rom_state,
      const std::vector<double>& max_swing_distance,
      bool start_with_left_stance, bool zero_touchdown_impact,
      const std::set<int>& relax_index, const PlannerSetting& param,
      const std::vector<int>& num_time_samples_ds,
      bool start_in_double_support_phase,
      const std::set<int>& idx_constant_rom_vel_during_double_support,
      bool print_status = true);

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
};

}  // namespace goldilocks_models
}  // namespace dairlib
