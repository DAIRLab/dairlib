#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"
#include "drake/solvers/gurobi_solver.h"

#include "solvers/fcc_qp_solver.h"

namespace dairlib::systems::controllers {

class SwingFootTrajSolver {
  static constexpr int kPolyDegZ = 10;
  static constexpr int kPolyDegXY = 7;

 public:

  // TODO (@Brian-Acosta) this might be more numerically stable by using an
  //  orthonormal polynomial basis
  explicit SwingFootTrajSolver();

  // Find a swing foot trajectory with continuous accelerations given by a 9th
  // order polynomial, similarly to https://arxiv.org/pdf/1704.01271v1.pdf
  // (equations 23-25), but with an additional cost on the mid-stride x/y
  // position to accommodate stepping up/down
  drake::trajectories::PathParameterizedTrajectory<double> AdaptSwingFootTraj(
      const drake::trajectories::PathParameterizedTrajectory<double>& prev_traj,
      double prev_time,
      double t_start,
      double t_end,
      double clearance,
      double z_vel_final,
      double z_pos_final_offset,
      const Eigen::Vector3d& initial_pos,
      const Eigen::Vector3d& target);

 private:

  drake::trajectories::PathParameterizedTrajectory<double> ConvertSolutionToTrajectory(
      const drake::solvers::MathematicalProgramResult& result,
      double t_start, double t_end) const;

  void UpdateContinuityConstraintsWithSlack();

  static Eigen::Vector3d CalcDesiredMidpoint(const Eigen::Vector3d& initial_pos,
                                      const Eigen::Vector3d& target,
                                      double clearance);

  drake::solvers::MathematicalProgram prog_{};

  Eigen::MatrixXd min_acc_Q_;

  drake::solvers::VectorXDecisionVariable cx_;
  drake::solvers::VectorXDecisionVariable cy_;
  drake::solvers::VectorXDecisionVariable cz_;
  drake::solvers::VectorXDecisionVariable cont_slack_;

  solvers::FCCQPSolver solver_;

  std::vector<drake::solvers::VectorXDecisionVariable> dim_var_map_;
  std::vector<std::vector<Eigen::RowVectorXd>> knot_deriv_multipliers_;
  std::vector<std::vector<Eigen::Vector3d>> knot_deriv_rhs_;

  std::vector<drake::solvers::LinearEqualityConstraint*> x_knot_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> y_knot_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> z_knot_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> x_end_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> y_end_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> z_end_constraints_;

  std::shared_ptr<drake::solvers::QuadraticCost> x_min_accel_cost;
  std::shared_ptr<drake::solvers::QuadraticCost> y_min_accel_cost;
  std::shared_ptr<drake::solvers::QuadraticCost> z_min_accel_cost;
  std::shared_ptr<drake::solvers::QuadraticCost> midpoint_target_cost_;
  std::shared_ptr<drake::solvers::QuadraticCost> cont_slack_cost_;

};

}