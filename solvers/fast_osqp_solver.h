#pragma once

#include <osqp.h>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solver_base.h"

namespace dairlib {
namespace solvers {
/**
 * This class is a slight modification of Drake's OsqpSolver that saves the
 * osqp_settings_ and workspace_ along with warm starting the solve.
 * The OSQP solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<OsqpSolver>() to obtain the
 * details.
 */

/**
 * For a sparse(or dense) matrix, return a vector of triplets, such that we can
 * reconstruct the matrix using setFromTriplet function
 * @param matrix A sparse matrix
 * @return A triplet with the row, column and value of the non-zero entries.
 * See https://eigen.tuxfamily.org/dox/group__TutorialSparse.html for more
 * information on the triplet
 */
template <typename Derived>
std::vector<Eigen::Triplet<typename Derived::Scalar>>
SparseOrDenseMatrixToTriplets(
    const Derived& matrix) {
  using Scalar = typename Derived::Scalar;
  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(matrix.nonZeros());
  for (int i = 0; i < matrix.outerSize(); i++) {
    for (typename Derived::InnerIterator it(matrix, i); it; ++it) {
      triplets.push_back(
          Eigen::Triplet<Scalar>(it.row(), it.col(), it.value()));
    }
  }
  return triplets;
}

class FastOsqpSolver final : public drake::solvers::SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FastOsqpSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = drake::solvers::OsqpSolverDetails;

  FastOsqpSolver();
  ~FastOsqpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static drake::solvers::SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(
      const drake::solvers::MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(
      const drake::solvers::MathematicalProgram&);
  //@}

  void InitializeSolver(const drake::solvers::MathematicalProgram&,
                        const drake::solvers::SolverOptions&);

  /// Solver will automatically reenable warm starting after a successful solve
  void DisableWarmStart() const {
    osqp_settings_->warm_start = false;
    warm_start_ = false;
  }
  /// Solver will automatically reenable warm starting after a successful solve
  void EnableWarmStart() const {
    osqp_settings_->warm_start = true;
    warm_start_ = true;
  }

  void WarmStart(const Eigen::VectorXd& primal, const Eigen::VectorXd& dual);

  bool IsInitialized() const { return is_init_; }

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const drake::solvers::MathematicalProgram&,
               const Eigen::VectorXd&, const drake::solvers::SolverOptions&,
               drake::solvers::MathematicalProgramResult*) const final;

  OSQPData* osqp_data_;
  mutable csc* P_csc_ = nullptr;
  mutable csc* A_csc_ = nullptr;
  mutable Eigen::SparseMatrix<c_float> P_sparse_;
  mutable Eigen::SparseMatrix<c_float> A_sparse_;
  mutable std::vector<c_float> l_;
  mutable std::vector<c_float> u_;
  mutable std::vector<c_float> q_;
  mutable std::vector<Eigen::Triplet<c_float>> P_triplets_;
  mutable std::vector<Eigen::Triplet<c_float>> A_triplets_;

  mutable OSQPSettings* osqp_settings_;
  mutable OSQPWorkspace* workspace_;
  mutable bool warm_start_ = true;
  mutable bool is_init_ = false;
};
}  // namespace solvers
}  // namespace dairlib
