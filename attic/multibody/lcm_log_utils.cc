#include "attic/multibody/lcm_log_utils.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "systems/primitives/vector_aggregator.h"
#include "systems/robot_lcm_systems.h"

#include "dairlib/lcmt_robot_output.hpp"

namespace dairlib {
namespace multibody {

using std::cout;
using std::endl;
using std::string;

using Eigen::VectorXd;
using Eigen::MatrixXd;


void parseLcmOutputLog(const RigidBodyTree<double>& tree, string file,
    string channel, VectorXd* t, MatrixXd* x, MatrixXd* u, double duration) {

  using drake::systems::lcm::LcmSubscriberSystem;

  drake::lcm::DrakeLcmLog r_log(file, false);

  drake::systems::DiagramBuilder<double> builder;

  builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

  auto output_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>(channel, &r_log));

  auto output_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);
  builder.Connect(*output_sub, *output_receiver);

  auto output_aggregator = builder.AddSystem<systems::VectorAggregator>(
      output_receiver->get_output_port(0).size() - 1);

  builder.Connect(*output_receiver, *output_aggregator);

  auto diagram = builder.Build();

  drake::systems::Simulator<double> sim(*diagram);

  // 1000 is the duration to playback (in seconds)
  sim.StepTo(r_log.GetNextMessageTime() + duration);

  *t = output_aggregator->BuildTimestampVector();

  auto data = output_aggregator->BuildMatrixFromVectors();
  *x = data.block(0, 0, tree.get_num_positions() + tree.get_num_velocities(),
      data.cols());
  *u = data.block(tree.get_num_positions() + tree.get_num_velocities(), 0,
      tree.get_num_actuators(), data.cols());
}

}  // namespace multibody
}  // namespace dairlib
