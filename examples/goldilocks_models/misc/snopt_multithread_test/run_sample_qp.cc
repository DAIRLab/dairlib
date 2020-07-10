#include "examples/goldilocks_models/misc/snopt_multithread_test/run_sample_qp.h"

#include <string>

#include "common/file_utils.h"
#include "drake/solvers/choose_best_solver.h"

using drake::AutoDiffXd;
using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>

namespace dairlib {
namespace goldilocks_models {
namespace misc {

using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinConstraintType;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::HybridDircon;

void runSampleQp(std::string directory, std::string prefix) {
  // Initialize the matrix size
  MatrixXd H_o = MatrixXd::Zero(2, 2);
  VectorXd b_o = VectorXd::Zero(2);
  VectorXd c_o = VectorXd::Zero(1);
  MatrixXd A_o = MatrixXd::Zero(2, 2);
  VectorXd lb_o = VectorXd::Zero(2);
  VectorXd ub_o = VectorXd::Zero(2);

  // An example
  H_o << 1.36075, 0.354964, 0.354964, 1.19376;
  b_o = VectorXd::Ones(2);
  c_o << 0;
  A_o << 0.411647, -0.164777, -0.302449, 0.26823;
  lb_o = 0.5 * VectorXd::Ones(2);
  ub_o = VectorXd::Ones(2);

  // solve optimization problem
  MathematicalProgram nlprog;
  auto w = nlprog.NewContinuousVariables(2, "w");
  // intentionally split the following constraint into two constraint bindings
  nlprog.AddLinearConstraint(A_o.row(0), lb_o(0), ub_o(0), w);
  nlprog.AddLinearConstraint(A_o.row(1), lb_o(1), ub_o(1), w);
  // Adds a cost term of the form 0.5*x'*H_o*x + b_o'x + c
  // intentionally split the following cost into two cost bindings
  nlprog.AddQuadraticCost(H_o * 1 / 3, b_o * 1 / 3, w);
  nlprog.AddQuadraticCost(H_o * 2 / 3, b_o * 2 / 3, w);

  // nlprog.SetSolverOption(drake::solvers::GurobiSolver::id(), "BarConvTol",
  // 1E-9);
  nlprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                         "snopt.out");
  nlprog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                         "Major iterations limit", 10000);
  // nlprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major
  // feasibility tolerance", 1.0e-14); //1.0e-10
  // nlprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor
  // feasibility tolerance", 1.0e-14); //1.0e-10

  // Testing
  cout << "Choose the best solver: "
       << drake::solvers::ChooseBestSolver(nlprog).name() << endl;

  cout << prefix << " starts solving...\n";
  const auto result = Solve(nlprog);
  auto solution_result = result.get_solution_result();
  cout << prefix << " " << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << " | ";
  VectorXd w_sol = result.GetSolution(nlprog.decision_variables());
  cout << "w_sol norm:" << w_sol.norm() << endl;

  // Check which solver we are using
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // Store a bool indicating whehter the problem was solved.
  VectorXd is_success(1);
  if (result.is_success())
    is_success << 1;
  else
    is_success << 0;
  writeCSV(directory + prefix + string("is_success.csv"), is_success);

  // Store the vectors and matrices
  // cout << "\nStoring vectors and matrices into csv.\n";
  double c_double = result.get_optimal_cost();
  VectorXd c(1);
  c << c_double;
  writeCSV(directory + prefix + string("c.csv"), c);

  return;
}

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib
