#include <string>
#include "attic/multibody/lcm_log_utils.h"
#include "examples/Cassie/cassie_utils.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

int main() {
  VectorXd t;
  MatrixXd x, u;

  RigidBodyTree<double> tree;
  dairlib::buildCassieTree(tree);
  std::string channel = "CASSIE_STATE";
  std::string file = "/home/posa/workspace/logs/constrained_lqr.log";
  dairlib::multibody::parseLcmOutputLog(tree, file, channel, &t, &x, &u);

  std::cout << "*****t*****" << std::endl;
  std:: cout << t << std::endl << std::endl;

  std::cout << "*****x*****" << std::endl;
  std:: cout << x << std::endl << std::endl;

  std::cout << "*****u*****" << std::endl;
  std:: cout << u << std::endl << std::endl;

  return 0;
}
