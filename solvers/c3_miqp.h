#pragma once
#include <vector>
#include <Eigen/Dense>

#include "solvers/c3.h"

namespace dairlib {
namespace solvers {

class C3MIQP:C3 {
 public:
	/// Default constructor for time-varying LCS
  /// TODO: add documentation
	C3MIQP(const std::vector<Eigen::MatrixXd>& A, const std::vector<Eigen::MatrixXd>& B,
		     const std::vector<Eigen::MatrixXd>& D, const std::vector<Eigen::VectorXd>& d,
		     const std::vector<Eigen::MatrixXd>& E, const std::vector<Eigen::MatrixXd>& F,
         const std::vector<Eigen::MatrixXd>& H, const std::vector<Eigen::VectorXd>& c,
         const C3Options& options);

	/// Constructor for time-invariant LCS
	/// TODO: add documentation
  C3MIQP(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
  	     const Eigen::MatrixXd& D, const Eigen::VectorXd& d,
  	     const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
  	     const Eigen::MatrixXd& H, const Eigen::VectorXd& c, const int& N,
  	     const C3Options& options);


	/// TODO: determine inputs/outputs
	/// I just include one argument here as an example
	/// Virtual method, to be implemented by subclasses
  void SolveSingleProjection(const Eigen::MatrixXd& E);

 private:
  // TODO: add MIQP workspace variables and anything else needed

};

} // namespace dairlib
} // namespace solvers
