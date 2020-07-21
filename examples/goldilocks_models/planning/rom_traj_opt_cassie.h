#pragma once

#include <vector>
#include <memory.h>

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
class RomTrajOptCassie
    : public drake::systems::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RomTrajOptCassie)

  RomTrajOptCassie(
      std::vector<int> num_time_samples, std::vector<double> minimum_timestep,
      std::vector<double> maximum_timestep, Eigen::MatrixXd Q,
      Eigen::MatrixXd R, const ReducedOrderModel& rom,
      const drake::multibody::MultibodyPlant<double>& plant,
      bool zero_touchdown_impact, double desired_final_position,
      Eigen::VectorXd init_state, Eigen::VectorXd h_guess,
      Eigen::MatrixXd r_guess, Eigen::MatrixXd dr_guess,
      Eigen::MatrixXd tau_guess, Eigen::VectorXd x_guess_left_in_front,
      Eigen::VectorXd x_guess_right_in_front, bool with_init_guess,
      bool fix_duration, bool fix_all_timestep, bool add_x_pose_in_cost,
      bool straight_leg_cost);

  ~RomTrajOptCassie() override {}

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
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  x0_vars_by_mode(int mode) const;
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

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  const int num_modes_;
  const std::vector<int> mode_lengths_;
  std::vector<int> mode_start_;
  const drake::solvers::VectorXDecisionVariable z_post_impact_vars_;
  const drake::solvers::VectorXDecisionVariable x0_vars_;
  const drake::solvers::VectorXDecisionVariable xf_vars_;
  const int n_z_;
  const int n_x_;
  const drake::multibody::MultibodyPlant<double>& plant_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
