#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"

namespace dairlib {

using std::make_unique;
using std::move;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::geometry::SceneGraph;
using drake::systems::Simulator;

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::CassieOutputReceiver;
using dairlib::systems::CassieRbtStateEstimator;

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RigidBodyTree<double> tree;

  buildCassieTree(tree, "examples/Cassie/urdf/cassie_v2.urdf",
                  drake::multibody::joints::kQuaternion);
  const double terrain_size = 100;
  const double terrain_depth = 0.2;
  drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_output = "CASSIE_OUTPUT";
  const std::string channel_estimator = "CASSIE_ESTIMATOR";
  const int num_contacts = 4;

  // State Receiver
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Cassie out receiver
  auto cassie_output_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(channel_output, lcm));
  auto cassie_output_receiver = builder.AddSystem<CassieOutputReceiver>();
  builder.Connect(cassie_output_sub->get_output_port(),
                  cassie_output_receiver->get_input_port(0));

  // auto publisher =
  //    builder.AddSystem<drake::systems::DrakeVisualizer>(tree, lcm);

  // auto passthrough =
  // builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
  //    state_receiver->get_output_port(0).size(), 0,
  //    publisher->get_input_port(0).size());

  // builder.Connect(state_receiver->get_output_port(0),
  //                passthrough->get_input_port());
  // builder.Connect(passthrough->get_output_port(),
  // publisher->get_input_port(0));

  // publisher->set_publish_period(1.0 / 30.0);

  // Estimator
  // Initial state and bias values
  MatrixXd R_init = MatrixXd::Identity(3, 3);
  VectorXd v_init = VectorXd::Zero(3);
  VectorXd p_init = VectorXd::Zero(3);
  MatrixXd d_init = MatrixXd::Zero(3, 4);

  VectorXd ekf_x_init = VectorXd::Zero(27);
  for (int i = 0; i < 3; ++i) {
    ekf_x_init.segment(i * 3, 3) = R_init.row(i);
  }
  ekf_x_init.segment(9, 3) = v_init;
  ekf_x_init.segment(12, 3) = p_init;
  for (int i = 0; i < num_contacts; ++i) {
    ekf_x_init.segment(15 + i * 3, 3) = d_init.col(i);
  }

  std::cout << ekf_x_init << std::endl;
  VectorXd ekf_bias_init = VectorXd::Zero(6);
  MatrixXd P = 0.01 * MatrixXd::Identity(27, 27);

  auto estimator = builder.AddSystem<CassieRbtStateEstimator>(
      tree, ekf_x_init, ekf_bias_init, true, P);

  // Connecting the estimator
  builder.Connect(cassie_output_receiver->get_output_port(0),
                  estimator->get_input_port(0));

  // Publishing the estimator state
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      estimator->get_output_port(0).size(), 0,
      tree.get_num_positions() + tree.get_num_velocities());
  auto estimator_sender =
      builder.AddSystem<dairlib::systems::RobotOutputSender>(tree);
  auto estimator_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          channel_estimator, lcm, 1.0 / 30.0));
  builder.Connect(estimator->get_output_port(0), passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  estimator_sender->get_input_port(0));
  builder.Connect(estimator_sender->get_output_port(0),
                  estimator_publisher->get_input_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto stepper = make_unique<Simulator<double>>(*diagram, move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }

