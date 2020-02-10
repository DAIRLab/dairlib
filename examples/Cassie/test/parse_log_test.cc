#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/rigid_body_tree.h"

#include "systems/primitives/vector_aggregator.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

using std::string;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

namespace dairlib {
using systems::VectorAggregator;

int ParseLog(string filename) {
  RigidBodyTree<double> tree;
  buildCassieTree(tree);

  drake::lcm::DrakeLcmLog r_log(filename, false);

  drake::systems::DiagramBuilder<double> builder;

  builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>("CASSIE_STATE", &r_log));

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                    state_receiver->get_input_port(0));

  auto state_aggregator = builder.AddSystem<VectorAggregator>(
      state_receiver->get_output_port(0).size() - 1);

  builder.Connect(state_receiver->get_output_port(0),
                  state_aggregator->get_input_port(0));

  auto input_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_input>("CASSIE_INPUT", &r_log));

  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(tree);
  builder.Connect(input_sub->get_output_port(),
                    input_receiver->get_input_port(0));

  auto input_aggregator = builder.AddSystem<VectorAggregator>(
      input_receiver->get_output_port(0).size() - 1);

  builder.Connect(input_receiver->get_output_port(0),
                  input_aggregator->get_input_port(0));

  auto diagram = builder.Build();

  drake::systems::Simulator<double> sim(*diagram);

  // 1000 is the duration to playback (in seconds)
  sim.AdvanceTo(r_log.GetNextMessageTime() + 0.1);

  std::cout << "*****timestamps*****" << std::endl;
  const std::vector<double> timestamps =
      state_aggregator->get_received_timestamps();
  for (uint i = 0; i < timestamps.size(); i++) {
    std::cout << timestamps[i] << std::endl;
  }

  std::cout << "*****data*****" << std::endl;
  const std::vector<Eigen::VectorXd>& data =
      state_aggregator->get_received_vectors();
  for (uint i = 0; i < data.size(); i++) {
    std::cout << data[i] << std::endl << std::endl;
  }

  std::cout << "*****timestamp vector*****" << std::endl;
  std:: cout << state_aggregator->BuildTimestampVector() << std::endl;

  std::cout << "*****data matrix*****" << std::endl;
  std:: cout << state_aggregator->BuildMatrixFromVectors() << std::endl;

  std::cout << "*****timestamp vector*****" << std::endl;
  std:: cout << input_aggregator->BuildTimestampVector() << std::endl;

  std::cout << "*****data matrix*****" << std::endl;
  std:: cout << input_aggregator->BuildMatrixFromVectors() << std::endl;


  return 0;
}

}  // namespace dairlib

int main() { return dairlib::ParseLog("test.log"); }
