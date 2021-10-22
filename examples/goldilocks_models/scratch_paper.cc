#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>   // queue with feature of finding elements
#include <queue>   // First in first out
#include <thread>  // multi-threading
#include <tuple>
#include <utility>        // std::pair, std::make_pair
#include <Eigen/QR>       // CompleteOrthogonalDecomposition
#include <bits/stdc++.h>  // system call
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "common/file_utils.h"

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

namespace dairlib::goldilocks_models {

int run(int argc, char* argv[]) {
  /// Reshape
  //  VectorXd vec(6);
  //  vec << 0, 1, 2, 3, 4, 5;
  //  MatrixXd mat = Eigen::Map<MatrixXd>(vec.data(), 3, 2).topRows<2>();
  //  cout << mat << endl;
  //  cout << mat * Eigen::VectorXd::Ones(2) << endl;

  // ReadCSV
  MatrixXd csv = readCSV("../test.csv");
  cout << "csv.size() = " << csv.size() << endl;
  cout << "csv = " << csv << endl;

  return 0;
}  // int run

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::run(argc, argv);
}
