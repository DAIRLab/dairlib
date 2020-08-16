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

// Reduced order model
// z = [y; ydot]
// yddot = theta_yddot * phi_yddot + B * tau

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
             bool zero_touchdown_impact = true);

  ~RomTrajOpt() override {}

  // The argument `dt_0` is the first time step size which we don't want to
  // equalize to the rest of the time step. Default to negative (the argument is
  // not used)
  void AddTimeStepConstraint(std::vector<double> minimum_timestep,
                             std::vector<double> maximum_timestep,
                             bool fix_duration, double duration,
                             bool equalize_timestep_size, double dt_0 = -1);

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
  const int n_z_;
  const int n_x_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const ReducedOrderModel& rom_;
  bool start_with_left_stance_;
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
                          const Eigen::VectorXd& final_position,
                          int fisrt_mode_phase_index);
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
