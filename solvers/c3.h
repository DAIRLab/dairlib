#pragma once
#include <vector>
#include <Eigen/Dense>

#include "solvers/c3_options.h"

namespace dairlib {
namespace solvers {
class C3 {
 public:
	/// Default constructor for time-varying LCS
  /// TODO: add documentation
	C3(const std::vector<Eigen::MatrixXd>& A, const std::vector<Eigen::MatrixXd>& B,
		 const std::vector<Eigen::MatrixXd>& D, const std::vector<Eigen::VectorXd>& d,
		 const std::vector<Eigen::MatrixXd>& E, const std::vector<Eigen::MatrixXd>& F,
     const std::vector<Eigen::MatrixXd>& H, const std::vector<Eigen::VectorXd>& c,
     const C3Options& options);

	/// Constructor for time-invariant LCS
	/// TODO: add documentation
  C3(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
  	 const Eigen::MatrixXd& D, const Eigen::VectorXd& d,
  	 const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
  	 const Eigen::MatrixXd& H, const Eigen::VectorXd& c, const int& N,
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
  Eigen::VectorXd Solve(const Eigen::VectorXd& x0, const Eigen::VectorXd& z0,
  											const Eigen::VectorXd& delta0,
  											const Eigen::VectorXd& w0, Eigen::VectorXd* z,
  											Eigen::VectorXd* delta, Eigen::VectorXd* w);

  /// Solve a single ADMM step
  /// TODO: add documentation
  /// @param z The primal variables from the previous step
  /// @param delta The copy variables from the previous step
  /// @param w The scaled dual variables from the previous step
  /// @param z_n A pointer to the output primal variables 
  /// @param delta_n A pointer to the output copy variables
  /// @param w_n A pointer to the scaled dual variables
  void ADMMStep(const Eigen::VectorXd& z, const Eigen::VectorXd& delta,
  						  const Eigen::VectorXd& w, Eigen::VectorXd* z_n,
  						  Eigen::VectorXd* delta_n, Eigen::VectorXd* w_n);

  /// TODO: determine inputs/outputs
  void SolveQP();

  /// TODO: determine inputs/outputs
  void SolveProjection();

	/// TODO: determine inputs/outputs
	/// I just include one argument here as an example
	/// Virtual method, to be implemented by subclasses
  virtual void SolveSingleProjection(const Eigen::MatrixXd& E);

 private:
 	const std::vector<Eigen::MatrixXd> A_;
 	const std::vector<Eigen::MatrixXd> B_;
	const std::vector<Eigen::MatrixXd> D_;
	const std::vector<Eigen::VectorXd> d_;
	const std::vector<Eigen::MatrixXd> E_;
	const std::vector<Eigen::MatrixXd> F_;
  const std::vector<Eigen::MatrixXd> H_;
  const std::vector<Eigen::VectorXd> c_;
  const C3Options options_;
  const int N_;

  // TODO: add QP workspace variables and anything else needed

};

} // namespace dairlib
} // namespace solvers
