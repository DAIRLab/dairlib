#pragma once
#include <vector>
#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "solvers/lcs.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {
class C3 {
 public:

    /// @param LCS LCS parameters
    /// @param Q, R, G, U Cost function parameters
    C3(const LCS& LCS, const std::vector<Eigen::MatrixXd>& Q, const std::vector<Eigen::MatrixXd>& R, const std::vector<Eigen::MatrixXd>& G, const std::vector<Eigen::MatrixXd>& U,
       const C3Options& options);

    /// Solve the MPC problem
    /// @param x0 The initial state of the system
    /// @param delta A pointer to the copy variable solution
    /// @param w A pointer to the scaled dual variable solution
    /// @return The first control action to take, u[0]
    Eigen::VectorXd Solve(Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& delta, std::vector<Eigen::VectorXd>& w);

    /// Solve a single ADMM step
    /// @param x0 The initial state of the system
    /// @param delta The copy variables from the previous step
    /// @param w The scaled dual variables from the previous step
    /// @param G A pointer to the G variables from previous step
    Eigen::VectorXd ADMMStep(Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta, std::vector<Eigen::VectorXd>* w, std::vector<Eigen::MatrixXd>* G);

    /// Solve a single QP
    /// @param x0 The initial state of the system
    /// @param WD A pointer to the (w - delta) variables
    /// @param G A pointer to the G variables from previous step
    std::vector<Eigen::VectorXd> SolveQP(Eigen::VectorXd& x0, std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WD);

    /// Solve the projection problem for all timesteps
    /// @param WZ A pointer to the (z + w) variables
    /// @param G A pointer to the G variables from previous step
    std::vector<Eigen::VectorXd> SolveProjection(std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WZ );

    /// Solve a single projection step
    /// @param E, F, H, c LCS parameters
    /// @param U A pointer to the U variables
    /// @param delta_c A pointer to the copy of (z + w) variables
    virtual Eigen::VectorXd SolveSingleProjection(const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c, const Eigen::MatrixXd& E, const Eigen::MatrixXd& F, const Eigen::MatrixXd& H, const Eigen::VectorXd& c) = 0;

 public:
 	const std::vector<Eigen::MatrixXd> A_;
 	const std::vector<Eigen::MatrixXd> B_;
	const std::vector<Eigen::MatrixXd> D_;
	const std::vector<Eigen::VectorXd> d_;
	const std::vector<Eigen::MatrixXd> E_;
	const std::vector<Eigen::MatrixXd> F_;
    const std::vector<Eigen::MatrixXd> H_;
    const std::vector<Eigen::VectorXd> c_;
    const std::vector<Eigen::MatrixXd> Q_;
    const std::vector<Eigen::MatrixXd> R_;
    const std::vector<Eigen::MatrixXd> U_;
    const std::vector<Eigen::MatrixXd> G_;
  const C3Options options_;
  const int N_;
  const int n_;
  const int m_;
  const int k_;
  const bool hflag_;

};

} // namespace dairlib
} // namespace solvers
