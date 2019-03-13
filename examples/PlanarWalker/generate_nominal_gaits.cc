#include <memory>
#include <chrono>
#include <random>
#include <gflags/gflags.h>


#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

#include "sgd_iter.h"
#include "systems/goldilocks_models/file_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::solvers::VectorXDecisionVariable;

namespace dairlib {
namespace goldilocks_models {

void generateNominalGaits() {
  std::random_device randgen;
  std::default_random_engine e1(randgen());
  std::uniform_real_distribution<> dist(0, 1);

  // int n_weights = 43;
  int n_weights =  10;
  MatrixXd theta_0 = MatrixXd::Zero(2,n_weights);
  theta_0(0,0) = -0.1;
  theta_0(0,3) = 1.0;
  theta_0(1,0) = 0;
  theta_0(1,1) = 1;
  writeCSV("data/0_theta.csv", theta_0);

  double length;
  double duration = 1;
  int snopt_iter = 200;
  string directory = "data/";
  string init_z = "z_save.csv";
  string weights = "0_theta.csv";
  string output_prefix;
  VectorXd lengths(5);
  lengths << .35, .3, .25, .2, .15;

  for (int i = 0; i < lengths.size(); i++) {
    length = lengths(i);
    // duration = length/speed;
    duration = 1;
    output_prefix = "init_length_" + std::to_string(i) + "_speed_" + std::to_string(0) + "_";
    sgdIter(length, duration, snopt_iter, directory, init_z, weights, output_prefix);
    init_z = output_prefix + "z.csv";
  }
}
}
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  dairlib::goldilocks_models::generateNominalGaits();
}
