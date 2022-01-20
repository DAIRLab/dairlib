#pragma once
#include <vector>
#include <Eigen/Dense>

#include "solvers/c3_options.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {
class C3 {
 public:
	/// Default constructor for time-varying LCS
  /// @param A, B, D, d Dynamics constraints x_{k+1} = A_k x_k + B_k u_k + D_k \lambda_k + d_k
  /// @param E, F, H, c Complementarity constraints  0 <= \lambda_k \perp E_k x_k + F_k \lambda_k  + H_k u_k + c_k
	C3(const std::vector<Eigen::MatrixXd>& A, const std::vector<Eigen::MatrixXd>& B,
		 const std::vector<Eigen::MatrixXd>& D, const std::vector<Eigen::MatrixXd>& d,
		 const std::vector<Eigen::MatrixXd>& E, const std::vector<Eigen::MatrixXd>& F,
     const std::vector<Eigen::MatrixXd>& H, const std::vector<Eigen::VectorXd>& c, const std::vector<Eigen::MatrixXd>& Q, const std::vector<Eigen::MatrixXd>& R, const std::vector<Eigen::MatrixXd>& G,
     const C3Options& options);

	/// Constructor for time-invariant LCS
	/// TODO: add documentation
  C3(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
  	 const Eigen::MatrixXd& D, const Eigen::MatrixXd& d,
  	 const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
  	 const Eigen::MatrixXd& H, const Eigen::VectorXd& c, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& G, const int& N,
  	 const C3Options& options);

  /// Solve the MPC problem
  /// TODO: add documentation
  /// TODO: I'm not really sure what inputs are needed here
  ///       It's probably some subset of what I include below
  ///       I'm also not sure whether you'd rather provide the more human
  ///       readable MatrixXd (n_x by T) or as a vector (n_x*T by 1)
  ///       Treat this as just one possibil
  /// @param x0 The initial state of the system
  /// @param z0 A guess for the primal decision variables
  /// @param delta0 A guess for the copy variables delta
  /// @param w A guess for the scaled dual variables w 
  /// @param z A pointer to the primal solution
  /// @param delta A pointer to the copy variable solution
  /// @param w A pointer to the scaled dual variable solution
  /// @return The first control action to take, u[0]
  Eigen::VectorXd Solve(Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta, std::vector<Eigen::VectorXd>* w);

  /// Solve a single ADMM step
  /// TODO: add documentation
  /// @param z The primal variables from the previous step
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param z_n A pointer to the output primal variables 
  /// @param delta_n A pointer to the output copy variables
  /// @param w_n A pointer to the scaled dual variables
  void ADMMStep(Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>* delta, std::vector<Eigen::VectorXd>* w, std::vector<Eigen::MatrixXd>* G);

  /// TODO: determine inputs/outputs
  std::vector<Eigen::VectorXd> SolveQP(Eigen::VectorXd& x0, std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WD);

  /// TODO: determine inputs/outputs
  std::vector<Eigen::VectorXd> SolveProjection(std::vector<Eigen::MatrixXd>& G, std::vector<Eigen::VectorXd>& WZ );

	/// TODO: determine inputs/outputs
	/// I just include one argument here as an example
	/// Virtual method, to be implemented by subclasses
  virtual Eigen::VectorXd SolveSingleProjection(const Eigen::MatrixXd& U, const Eigen::VectorXd& delta_c, const Eigen::MatrixXd& E, const Eigen::MatrixXd& F, const Eigen::MatrixXd& H, const Eigen::VectorXd& c) = 0;

 public:
 	const std::vector<Eigen::MatrixXd> A_;
 	const std::vector<Eigen::MatrixXd> B_;
	const std::vector<Eigen::MatrixXd> D_;
	const std::vector<Eigen::MatrixXd> d_;
	const std::vector<Eigen::MatrixXd> E_;
	const std::vector<Eigen::MatrixXd> F_;
    const std::vector<Eigen::MatrixXd> H_;
    const std::vector<Eigen::VectorXd> c_;
    const std::vector<Eigen::MatrixXd> Q_;
    const std::vector<Eigen::MatrixXd> R_;
    const std::vector<Eigen::MatrixXd> G_;
  const C3Options options_;
  const int N_;
  const int n_;
  const int m_;
  const int k_;
  const bool hflag_;

  // TODO: add QP workspace variables and anything else needed

};

} // namespace dairlib
} // namespace solvers
