#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"

#include "attic/systems/trajectory_optimization/dircon_util.h"

using drake::systems::trajectory_optimization::dircon::checkConstraints;
using drake::systems::trajectory_optimization::dircon::linearizeConstraints;
using std::cout;
using std::endl;

int main(int argc, char* argv[]) {

  drake::solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(5);
  auto y = prog.NewContinuousVariables(3);

  MatrixXd A_lin(2,6);
  A_lin << 1,2,3,4,5,6,7,8,9,10,11,12;
  VectorXd b_lin(2);
  b_lin << -100,-200;
  drake::solvers::VectorXDecisionVariable tmp(6);
  tmp << x(0), x(1), x(4), x(3), y(1), y(2);

  prog.AddLinearConstraint(A_lin*tmp == b_lin);
  prog.AddLinearConstraint(A_lin*tmp <= b_lin);

  prog.AddConstraint(x(2)*x(2) + y(2)*x(1)*x(2) == 0);


  checkConstraints(&prog);

  MatrixXd A;
  VectorXd f,lb,ub;
  VectorXd z(8);
  z << -10,-9,-8,-7,-6,-5,-4,-3;
  linearizeConstraints(&prog, z, f, A, lb, ub);

  MatrixXd y_and_bounds(f.size(),3);
  y_and_bounds.col(0) = lb;
  y_and_bounds.col(1) = f;
  y_and_bounds.col(2) = ub;
  cout << "*************[lb f ub]***************" << endl;
  cout << y_and_bounds << endl;
  cout << "*************A***************" << endl;
  cout << A << endl;

}