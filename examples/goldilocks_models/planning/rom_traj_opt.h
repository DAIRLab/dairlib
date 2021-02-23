#pragma once

#include <tuple>
#include <vector>
#include <memory.h>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

#include "examples/goldilocks_models/reduced_order_models.h"

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

  RomTrajOpt(std::vector<int> num_time_samples, Eigen::MatrixXd Q,
             Eigen::MatrixXd R, const ReducedOrderModel& rom,
             const drake::multibody::MultibodyPlant<double>& plant,
             const StateMirror& state_mirror,
             const std::vector<BodyPoint>& left_contacts,
             const std::vector<BodyPoint>& right_contacts,
             const std::vector<std::tuple<std::string, double, double>>&
                 fom_joint_name_lb_ub,
             Eigen::VectorXd x_init, bool start_with_left_stance,
             bool zero_touchdown_impact, bool print_status = true);

  ~RomTrajOpt() override {}

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

  drake::VectorX<drake::symbolic::Expression> SubstitutePlaceholderVariables(
      const drake::VectorX<drake::symbolic::Expression>& f,
      int interval_index) const;

  int num_knots() const {
    return std::accumulate(mode_lengths_.begin(), mode_lengths_.end(), 0) -
           mode_lengths_.size() + 1;
  }
  int num_modes() const { return num_modes_; }
  const std::vector<int>& mode_start() const { return mode_start_; }

  const ReducedOrderModel& reduced_order_model() const { return rom_; }

 protected:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  const int num_modes_;
  const std::vector<int> mode_lengths_;
  std::vector<int> mode_start_;
  const drake::solvers::VectorXDecisionVariable z_post_impact_vars_;
  const drake::solvers::VectorXDecisionVariable x0_var_;
  const drake::solvers::VectorXDecisionVariable xf_vars_;
  const drake::solvers::VectorXDecisionVariable v_post_impact_vars_;
  const int n_y_;
  const int n_z_;
  const int n_x_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const ReducedOrderModel& rom_;
  bool start_with_left_stance_;

  void PrintStatus(const std::string& msg) const {
    if (print_status_) std::cout << msg << std::endl;
  };
  bool print_status_;
};

class RomTrajOptCassie : public RomTrajOpt {
 public:
  RomTrajOptCassie(std::vector<int> num_time_samples, Eigen::MatrixXd Q,
                   Eigen::MatrixXd R, const ReducedOrderModel& rom,
                   const drake::multibody::MultibodyPlant<double>& plant,
                   const StateMirror& state_mirror,
                   const std::vector<BodyPoint>& left_contacts,
                   const std::vector<BodyPoint>& right_contacts,
                   const std::vector<std::tuple<std::string, double, double>>&
                       fom_joint_name_lb_ub,
                   Eigen::VectorXd x_init, bool start_with_left_stance,
                   bool zero_touchdown_impact, bool print_status = true);

  void AddRegularizationCost(const Eigen::VectorXd& final_position,
                             const Eigen::VectorXd& x_guess_left_in_front,
                             const Eigen::VectorXd& x_guess_right_in_front,
                             double w_reg_quat, double w_reg_xy,
                             double w_reg_z_joints, bool straight_leg_cost);

  void SetHeuristicInitialGuess(const Eigen::VectorXd& h_guess,
                                const Eigen::MatrixXd& r_guess,
                                const Eigen::MatrixXd& dr_guess,
                                const Eigen::MatrixXd& tau_guess,
                                const Eigen::VectorXd& x_guess_left_in_front,
                                const Eigen::VectorXd& x_guess_right_in_front,
                                const Eigen::VectorXd& final_position,
                                int fisrt_mode_phase_index,
                                int starting_mode_index);

  // Testing -- AddRomRegularizationCost is SetAllInitialGuess except we replace
  // setting guess with setting cost
  void AddRomRegularizationCost(const Eigen::VectorXd& h_guess,
                                const Eigen::MatrixXd& r_guess,
                                const Eigen::MatrixXd& dr_guess,
                                const Eigen::MatrixXd& tau_guess,
                                int fisrt_mode_phase_index, double w_reg);
};

class RomTrajOptFiveLinkRobot : public RomTrajOpt {
 public:
  RomTrajOptFiveLinkRobot(
      std::vector<int> num_time_samples, Eigen::MatrixXd Q, Eigen::MatrixXd R,
      const ReducedOrderModel& rom,
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
