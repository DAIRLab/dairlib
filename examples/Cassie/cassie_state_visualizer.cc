#include <gflags/gflags.h>

#include "systems/primitives/subvector_pass_through.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/simulator_drift.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {

DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_double(drift_rate, 0.0, "Drift rate for simulator");
DEFINE_string(channel, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::systems::Simulator;
using std::cout;
using std::endl;

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RigidBodyTree<double> tree;

  if (!FLAGS_floating_base) {
    buildCassieTree(tree);
  } else {
    buildCassieTree(tree, "examples/Cassie/urdf/cassie_v2.urdf",
                    drake::multibody::joints::kQuaternion);
    // const double terrain_size = 100;
    // const double terrain_depth = 0.20;
    // drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size,
    // terrain_depth);
  }

  drake::systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto state_sub =
      builder.AddSystem(drake::systems::lcm::LcmSubscriberSystem::Make<
                        dairlib::lcmt_robot_output>(FLAGS_channel, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
      state_receiver->get_input_port(0));

  Eigen::VectorXd drift_mean = Eigen::VectorXd::Zero(tree.get_num_positions());
  Eigen::MatrixXd drift_cov =
      Eigen::MatrixXd::Zero(tree.get_num_positions(), tree.get_num_positions());

  drift_mean(0) = FLAGS_drift_rate; //x
  drift_mean(1) = FLAGS_drift_rate; //y
  drift_mean(2) = FLAGS_drift_rate; //z

  drift_cov(0,0) = FLAGS_drift_rate;
  drift_cov(1,1) = FLAGS_drift_rate;
  drift_cov(2,2) = FLAGS_drift_rate;

  auto simulator_drift =
      builder.AddSystem<SimulatorDrift>(tree, drift_mean, drift_cov);

  auto publisher =
      builder.AddSystem<drake::systems::DrakeVisualizer>(tree, lcm);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      state_receiver->get_output_port(0).size(), 0,
      publisher->get_input_port(0).size());

//  builder.Connect(state_receiver->get_output_port(0),
//                  passthrough->get_input_port());
  builder.Connect(state_receiver->get_output_port(0),
      simulator_drift->get_input_port_state());
  builder.Connect(simulator_drift->get_output_port(0),
      passthrough->get_input_port());

  builder.Connect(passthrough->get_output_port(), publisher->get_input_port(0));

  publisher->set_publish_period(1.0 / 30.0);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  std::cout << "Built diagram" << std::endl;

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }
