#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"

#include "solvers/fcc_qp_solver.h"

namespace dairlib::systems::controllers {

class SwingFootTrajSolver {
  static constexpr int kSwingPolyDeg = 10;
  using RowVectorkSwingPolyDegd = drake::RowVector<double, kSwingPolyDeg>;

 public:
  explicit SwingFootTrajSolver();

  // Find a swing foot trajectory with continuous accelerations given by a 9th
  // order polynomial, similarly to https://arxiv.org/pdf/1704.01271v1.pdf
  // (equations 23-25), but with an additional cost on the mid-stride x/y
  // position to accommodate stepping up/down
  drake::trajectories::PathParameterizedTrajectory<double> AdaptSwingFootTraj(
      const drake::trajectories::PathParameterizedTrajectory<double>& prev_traj,
      double prev_time, double t_start, double t_end,
      double swing_foot_clearance, double z_vel_final, double z_pos_final_offset,
      const Eigen::Vector3d& initial_pos, const Eigen::Vector3d& footstep_target);

 private:
  const double conditioning_offset_ = 0.25;

  drake::solvers::MathematicalProgram prog_{};

  drake::solvers::VectorXDecisionVariable cx_;
  drake::solvers::VectorXDecisionVariable cy_;
  drake::solvers::VectorXDecisionVariable cz_;

  solvers::FCCQPSolver solver_;

  std::vector<drake::solvers::VectorXDecisionVariable> dim_var_map_;
  std::vector<std::vector<RowVectorkSwingPolyDegd>> knot_deriv_multipliers_;
  std::vector<std::vector<Eigen::Vector3d>> knot_deriv_rhs_;
  std::vector<std::vector<std::vector<drake::solvers::LinearEqualityConstraint*>>> knot_constraints_;

  std::shared_ptr<drake::solvers::QuadraticCost> midpoint_target_cost_;
};

}