#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>   // queue with feature of finding elements
#include <queue>   // First in first out
#include <thread>  // multi-threading
#include <tuple>
#include <utility>  // std::pair, std::make_pair

#include <Eigen/QR>       // CompleteOrthogonalDecomposition
#include <bits/stdc++.h>  // system call
#include <gflags/gflags.h>

#include "common/file_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::VectorXcd;
using Eigen::VectorXd;
using std::cin;
using std::cout;
using std::endl;
using std::pair;
using std::string;
using std::to_string;
using std::vector;

using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib::goldilocks_models {

int run(int argc, char* argv[]) {
  /// Reshape
  //  VectorXd vec(6);
  //  vec << 0, 1, 2, 3, 4, 5;
  //  MatrixXd mat = Eigen::Map<MatrixXd>(vec.data(), 3, 2).topRows<2>();
  //  cout << mat << endl;
  //  cout << mat * Eigen::VectorXd::Ones(2) << endl;

  // ReadCSV
  //  MatrixXd csv = readCSV("../test.csv");
  //  cout << "csv.size() = " << csv.size() << endl;
  //  cout << "csv = " << csv << endl;

  // PiecewisePolynomial
  // PiecewisePolynomial "evaluates from the right" when it's at break point.
  /*double eps = 1e-8;
  PiecewisePolynomial<double> pp;

  vector<double> breaks = {0, 1, 2};
  MatrixXd one = MatrixXd::Ones(1, 1);
  vector<MatrixXd> values = {one, 2 * one, 1 * one};
  pp.ConcatenateInTime(
      PiecewisePolynomial<double>::FirstOrderHold(breaks, values));
  cout << pp.value(-0.5) << endl;
  cout << pp.value(0) << endl;
  cout << pp.value(0.5) << endl;
  cout << pp.value(1) << endl;
  cout << pp.value(1.5) << endl;
  cout << pp.value(2) << endl;
  cout << pp.value(2.5) << endl;

  double break_eps = 1e-16;
  breaks = {2 + break_eps, 3, 4};
  values = {MatrixXd::Zero(1, 1), one, MatrixXd::Zero(1, 1)};
  pp.ConcatenateInTime(
      PiecewisePolynomial<double>::FirstOrderHold(breaks, values));
  cout << pp.value(2 - eps) << endl;
  cout << pp.value(2) << endl;
  cout << pp.value(2 + eps) << endl;*/

  return 0;
}  // int run

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::run(argc, argv);
}
