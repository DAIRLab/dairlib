#include <string>
#include "systems/log_parser/generic_lcm_log_parser.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

int main() {
  Eigen::VectorXd t;
  Eigen::MatrixXd x;

  RigidBodyTree<double> tree;
  dairlib::buildCassieTree(tree);
  std::string channel = "CASSIE_INPUT";
  std::string filename =
    "/home/nanda/DAIR/dairlib/examples/Cassie/lcmlog-2019-05-21.02";

  std::unique_ptr<dairlib::systems::RobotInputReceiver> system =
      std::make_unique<dairlib::systems::RobotInputReceiver>(tree);
  dairlib::multibody::parseLcmLog<dairlib::lcmt_robot_input>(
      std::move(system), filename, channel, &t, &x, 2.0e6);

  std::cout << "*****t*****" << std::endl;
  std:: cout << t.rows() << " " << t.cols() << std::endl << std::endl;

  std::cout << "*****x*****" << std::endl;
  std:: cout << x.rows() << " " << x.cols() << std::endl << std::endl;

  return 0;
}
