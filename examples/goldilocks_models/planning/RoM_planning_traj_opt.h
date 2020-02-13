#pragma once

#include <memory.h>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/multibody/plant/multibody_plant.h"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::multibody::MultibodyPlant;

namespace dairlib {
namespace goldilocks_models {

// Reduced order model
// y = [r; dr]
// ddr = theta_ddr * phi_ddr + B * tau

// Modified from HybridDircon class
class RomPlanningTrajOptWithFomImpactMap :
  public drake::systems::trajectory_optimization::MultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RomPlanningTrajOptWithFomImpactMap)

  RomPlanningTrajOptWithFomImpactMap(vector<int> num_time_samples,
                                     vector<double> minimum_timestep,
                                     vector<double> maximum_timestep,
                                     MatrixXd Q,
                                     MatrixXd R,
                                     int n_r,
                                     int n_tau,
                                     MatrixXd B_tau,
                                     int n_feature_kin,
                                     int n_feature_dyn,
                                     const VectorXd & theta_kin,
                                     const VectorXd & theta_dyn,
                                     const MultibodyPlant<double>& plant,
                                     bool zero_touchdown_impact,
                                     double desired_final_position,
                                     VectorXd init_state,
                                     VectorXd h_guess,
                                     MatrixXd r_guess,
                                     MatrixXd dr_guess,
                                     MatrixXd tau_guess,
                                     VectorXd x_guess_left_in_front,
                                     VectorXd x_guess_right_in_front,
                                     bool with_init_guess,
                                     bool fix_duration,
                                     bool fix_all_timestep,
                                     bool add_x_pose_in_cost,
                                     bool straight_leg_cost,
                                     int robot_option);

  ~RomPlanningTrajOptWithFomImpactMap() override {}

  /// Get the input trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
    const drake::solvers::MathematicalProgramResult& result) const override;

  /// Get the state trajectory at the solution as a
  /// %drake::trajectories::PiecewisePolynomialTrajectory%.
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
    const drake::solvers::MathematicalProgramResult& result) const override;

  const drake::solvers::VectorXDecisionVariable& y_post_impact_vars() const {
    return y_post_impact_vars_;
  }

  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  y_post_impact_vars_by_mode(int mode) const;
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  x0_vars_by_mode(int mode) const;
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  xf_vars_by_mode(int mode) const;

  /// Get the state decision variables given a mode and a time_index
  /// (time_index is w.r.t that particular mode). This will use the
  ///  y_post_impact_vars_ if needed. Otherwise, it just returns the standard
  /// x_vars element
  drake::solvers::VectorXDecisionVariable state_vars_by_mode(int mode,
      int time_index) const;

  drake::VectorX<drake::symbolic::Expression> SubstitutePlaceholderVariables(
    const drake::VectorX<drake::symbolic::Expression>& f,
    int interval_index) const;

 private:
  // Implements a running cost at all timesteps using trapezoidal integration.
  void DoAddRunningCost(const drake::symbolic::Expression& e) override;
  const int num_modes_;
  const std::vector<int> mode_lengths_;
  std::vector<int> mode_start_;
  const drake::solvers::VectorXDecisionVariable y_post_impact_vars_;
  const drake::solvers::VectorXDecisionVariable x0_vars_;
  const drake::solvers::VectorXDecisionVariable xf_vars_;
  const int n_r_;
  const int n_tau_;
  const int n_y_;
  const int n_x_;
  const drake::multibody::MultibodyPlant<double>& plant_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
