#include "qp_data.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;

using drake::solvers::MathematicalProgram;


void QPData::ToMathematicalProgram(MathematicalProgram& prog) const {
  DRAKE_DEMAND(ValidateDimensions());
  DRAKE_DEMAND(prog.num_vars() == 0);

  auto x = prog.NewContinuousVariables(num_vars);
  prog.AddQuadraticCost(MatrixXd(H), g, c, x);

  if (num_ineq > 0) {
    prog.AddLinearConstraint(A, lb, ub, x);
  }
  if (num_eq > 0) {
    prog.AddLinearEqualityConstraint(A_eq, b_eq, x);
  }
}

bool QPData::ValidateDimensions() const {
  DRAKE_DEMAND(H.rows() == H.cols());
  DRAKE_DEMAND(g.rows() == num_vars);
  DRAKE_DEMAND(H.rows() == num_vars);
  DRAKE_DEMAND(num_eq == A_eq.rows());
  DRAKE_DEMAND(num_ineq == A.rows());
  DRAKE_DEMAND(num_eq == b_eq.rows());
  DRAKE_DEMAND(num_ineq == lb.rows());
  DRAKE_DEMAND(num_ineq == ub.rows());
  return true;
}

}