#include <gflags/gflags.h>
#include <Eigen/Dense>
#include <string>

#include "systems/goldilocks_models/file_utils.h"
#include "drake/common/drake_assert.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;
using std::to_string;
using std::cin;
using std::cout;
using std::endl;

namespace dairlib {
namespace goldilocks_models {

DEFINE_int32(iter_start, 1, "start from which iteration");
DEFINE_int32(iter_end, -1, "end at which iteration");

DEFINE_int32(num_traj_opt_knots, 20, "# of traj opt knot points");
DEFINE_int32(num_batch, 9, "total number of batch");

DEFINE_double(weight_tau, 0.5, "The cost weight for tau");

// I only store the cost with tau, but didn't store one which doesn't include
// tau.
// This program creates the cost without tau.
int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_iter_end < 0) {
    cout << "Please enter iter_end.\n";
    return 0;
  }

  cout << "iter_start = " << FLAGS_iter_start << endl;
  cout << "iter_end = " << FLAGS_iter_end << endl;
  cout << "num_traj_opt_knots = " << FLAGS_num_traj_opt_knots << endl;
  cout << "num_batch = " << FLAGS_num_batch << endl;
  cout << "The cost weight for tau = " << FLAGS_weight_tau << endl;

  cout << "Warning: be careful that when you add cost, there is a 1/2 factor.\n";
  cout << "Are the above numbers correct? (Y/N)\n";
  char answer[1];
  cin >> answer;
  if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
    cout << "Ending the program, since the numbers are incorrect.\n";
    return 0;
  } else {
    cout << "Post-adding the cost...\n";
  }

  const string directory = "examples/goldilocks_models/find_models/data/";

  for (int i = FLAGS_iter_start; i <= FLAGS_iter_end; i++) {
    for (int j = 0; j < FLAGS_num_batch; j++) {
      string prefix = directory + to_string(i) + "_" + to_string(j) + "_";
      // Read in cost
      VectorXd c = readCSV(prefix + string("c.csv")).col(0);

      VectorXd tau_cost(1);
      tau_cost << 0;

      // Read in tau traj
      MatrixXd t_and_tau = readCSV(prefix + string("t_and_tau.csv"));
      if ((t_and_tau.cols() % FLAGS_num_traj_opt_knots) != 0) {
        cout << "You input a wrong num_traj_opt_knots.\n";
        return 0;
      }

      int n_tau = t_and_tau.rows() - 1;
      for (int k = 0; k < t_and_tau.cols(); k++) {
        VectorXd tau_k = t_and_tau.col(k).tail(n_tau);
        tau_cost += FLAGS_weight_tau * tau_k.transpose() * tau_k;
      }

      c -= tau_cost;

      // Store (overwrite) the parameters
      writeCSV(prefix + string("c_without_tau.csv"), c);
    }
  }

  cout << "Finished adding the cost without tau.\n";

  return 0;
}

}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::doMain(argc, argv);
  return 0;
}
